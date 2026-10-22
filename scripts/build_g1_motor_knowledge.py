#!/usr/bin/env python3
"""Build persistent motor facts from pinned official sources, without importing SDK control code."""
import ast
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.build_g1_capability_knowledge import build as capability_knowledge, service_audit
from scripts.g1_sport_evidence import service_readiness
from scripts.g1_humanoid_evidence import humanoid_review
from scripts.g1_publisher_evidence import publisher_review
from g1_console.state_learning import PROJECT_TILT_RAD, PROJECT_MOTOR_TEMP_C
from g1_console.config import Settings
from g1_console.contracts import MAX_VX, MAX_VY, MAX_OMEGA


# Photos and the vendor PDF are archived locally only (not in the public repository). Their digests stay pinned
# and are checked whenever the file is present; every other source must exist and match.
LOCAL_ONLY = re.compile(r'\.(png|jpe?g|webp|gif|pdf|mp4|mov)$', re.I)


def pinned(path, digest):
    file = ROOT/path
    if LOCAL_ONLY.search(path) and not file.exists():
        return
    assert hashlib.sha256(file.read_bytes()).hexdigest() == digest, path


def state_safety_knowledge(sources):
    text = (ROOT/sources['terminations']['path']).read_text()
    units = dict(bad_orientation='rad', joint_vel_out_of_limit='rad/s', ang_vel_out_of_limit='rad/s',
                 motor_winding_overheat='°C', motor_casing_overheat='°C', low_battery='%', lost_connection='ms')
    helpers = []
    for name, parameter, value in re.findall(r'inline bool (\w+)\([^)]*?\b(\w+)\s*=\s*([\d.]+)\)', text):
        helpers.append(dict(name=name, parameter=parameter, sdk_default=float(value), unit=units[name], source='terminations',
                            verified_device_threshold=None, execution_allowed=False))
    assert len(helpers) == 7
    settings = Settings()
    return dict(schema_version=1, authority='事实、参考默认值和项目拦截线分别保存；没有核定的防摔动作范围',
        sdk_helpers=helpers, sdk_default_meaning='可传参覆盖的 SDK 辅助函数默认值，不是本机固件阈值；未执行任何保护动作',
        project_guards=dict(telemetry_ttl_s=settings.telemetry_ttl, fsm_ttl_s=1.5,
            max_abs_roll_pitch_rad=PROJECT_TILT_RAD, motor_temperature_stop_c=PROJECT_MOTOR_TEMP_C,
            max_vx_m_s=MAX_VX, max_vy_m_s=MAX_VY, max_omega_rad_s=MAX_OMEGA,
            meaning='项目软件拦截条件；通过也不证明不会摔倒'),
        verified_motor_safety_limits=None, validated_action_envelopes=[], fall_free_guarantee=False,
        learning_policy=dict(strategy_learning_only=True, motion_model_training=False, passive_samples_can_expand_boundary=False,
            missing_or_stale_evidence='deny_motion', proposals_only=True),
        required_evidence=['匹配本机固件、坐标与标定的电机限值、健康和降额配置',
            '可信接触/载荷、全身状态、支撑关系及动态平衡裕量',
            '现场地面、摩擦、坡度、跌落边缘及全身净空',
            '每项官方能力完整进入/执行/退出/中断/恢复条件',
            '现场防跌落保护、官方平衡控制归属及失联/急停行为验证'],
        interpretation_rules=['GetFsmId 只确认回读模式；GetActionList 只声明能力及部分条件，均不证明具体动作执行中',
            '关节形态、IMU 与短时低速不能唯一确定站/坐/卧或足底接触',
            '当前观测窗口不自动变为安全范围；人工勾选地面条件不能替代验证',
            '当前真实动作范围为空；需另行实现并审查逐动作证据验证器，修改只读开关不放行运动'])


def safety_evidence_audit():
    evidence = json.loads((ROOT/'g1_console/assets/g1_safety_evidence_audit.json').read_text())
    for source in evidence['sources']:
        pinned(source['path'], source['sha256'])
    assert evidence['current'] is evidence['execution_allowed'] is False
    assert evidence['motor_configuration_values_obtained'] == evidence['command_writes'] == 0
    assert len(evidence['areas']) == 7 and not evidence['validated_action_envelopes']
    assert all(not area['safety_verified'] for area in evidence['areas'])
    return evidence


def standing_knowledge():
    python_path = 'externals/unitree_sdk2_python/unitree_sdk2py/g1/loco/g1_loco_client.py'
    cpp_path = 'g1_console/assets/safety_evidence_sources/g1_loco_client.hpp'
    tree = ast.parse((ROOT/python_path).read_text())
    methods = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    transitions = []
    for method, expected, meaning in [
        ('Squat2StandUp',706,'已确认蹲姿时的文档候选；与 StandUp2Squat 复用 706，不能当作幂等站起命令'),
        ('StandUp2Squat',706,'已确认站姿时的文档候选；与 Squat2StandUp 复用 706'),
        ('Lie2StandUp',702,'已确认卧姿时的文档候选；当前起始姿势未核实'),
        ('Start',500,'官方行走状态入口；不是站稳确认、起身轨迹或安全担保')]:
        calls = [n for n in ast.walk(methods[method]) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'SetFsmId']
        assert len(calls) == 1 and ast.literal_eval(calls[0].args[0]) == expected
        assert [arg.arg for arg in methods[method].args.args] == ['self']
        assert not methods[method].args.defaults and methods[method].args.kwarg is None
        transitions.append(dict(method=method, sdk='Python', fsm_id=expected, detail=meaning,
                                firmware_applicability_verified=None, execution_allowed=False))
    cpp = (ROOT/cpp_path).read_text()
    assert 'int32_t StandUp() { return SetFsmId(4); }' in cpp
    assert 'int32_t BalanceStand() { return SetBalanceMode(0); }' in cpp
    assert 'json.duration = duration;' in cpp and 'ROBOT_API_ID_LOCO_SET_VELOCITY' in cpp
    transitions.append(dict(method='StandUp', sdk='C++', fsm_id=4,
        detail='另一固定版本 C++ 包装为 4，与 Python 蹲站入口不同；不自动替换或试调用以判断兼容性',
        firmware_applicability_verified=None, execution_allowed=False))
    return dict(authority='固定官方 SDK 的能力语义与物理证据要求；不是实机动作参数或已验证站起策略',
        sources=[dict(path=p,sha256=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()) for p in (python_path,cpp_path)],
        documented_transitions=transitions, execution_allowed=False,
        support_audit=standing_support_audit(),
        required_evidence=[list(row) for row in [
            ('pose','起始姿势与支撑对象','需核实站/蹲/坐/卧、足底与座椅/吊带/人员等外部支撑，屈膝角度不唯一确定姿势'),
            ('load','双脚接触与承重','需匹配本机的接触与载荷传感器或经验证的估计器、单位/左右映射/质量与时效；不能由 tau_est 或四路零值替代'),
            ('balance','站稳与重心关系','需经验证的全身状态、支撑面、质心/动量与平衡裕量；允许误差、速度界限和持续时间均未核定'),
            ('controller','官方平衡控制与健康','CheckMode/FSM 一致只证明编号一致；需本机控制归属、驱动与电池健康、动作阶段及保护反馈'),
            ('transition','官方转换和现场条件','需匹配 23 DOF 本机固件的进入/执行/退出/失败恢复契约，以及现场地面、净空和防跌落保护')]],
        slow_stand_limitation='已核对的起身包装未公开起身时长/速度参数。SetVelocity.duration 是速度命令持续时间；遥操作手臂权重是手臂命令混合，均不作为全身起身减速参数。SetSpeedMode、SetStandHeight 或延时也未核定为起身减速控制；不生成低层关节插值，不用示例增益。',
        learning_steps=[
            dict(name='识别当前身体', detail='结合关节、机体朝向、可见身体和接触对象，先区分站/蹲/坐/卧；不能区分时继续观察。'),
            dict(name='辨认支撑与承重', detail='确认双脚和其他支撑，观察负载如何分配；不把关节力矩当作脚底力。'),
            dict(name='理解重心转移', detail='类比人起身，先理解支撑面和重心迁移的关系；当前没有校准证据，不计算可执行重心目标。'),
            dict(name='核定官方起身路径', detail='依据已确认的起始姿势选择可研究的官方转换；本机适用性、过程与慢速能力未确认时不选择执行路线。'),
            dict(name='确认站稳或继续观察', detail='同时核验姿势、接触承重、平衡控制、速度/裕量和持续时间；长时间静止、画面稳定或模式编号单独不构成站稳证据。')])


def standing_support_audit():
    """Timestamped human reports and official reads remain history, never live authority."""
    audit = json.loads((ROOT/'g1_console/assets/g1_standing_support_audit.json').read_text())
    for source in audit['sources']:
        pinned(source['path'], source['sha256'])
    assert audit['current'] is audit['execution_allowed'] is False
    assert audit['command_writes'] == audit['lease_requests'] == audit['motor_configuration_values_obtained'] == 0
    human = json.loads((ROOT/audit['human_source']).read_text())
    previous = human['supersedes']
    pinned(previous['path'], previous['sha256'])
    assert previous['field'] == 'reported_external_support'
    assert human[previous['field']] == previous['replacement']
    reads = json.loads((ROOT/audit['readings_source']).read_text())
    observation = json.loads((ROOT/audit['observation_source']).read_text())
    manual = json.loads((ROOT/audit['manual_source']).read_text())
    firmware = json.loads((ROOT/audit['firmware_source']).read_text())
    applicability = json.loads((ROOT/audit['applicability_source']).read_text())
    assert human['current'] is observation['current'] is reads['current'] is False
    assert human['observed_at_utc'] is human['left_load_n'] is human['right_load_n'] is None
    assert reads['rpc_calls'] == len(reads['readings']) == 6
    assert reads['command_writes'] == reads['lease_requests'] == reads['configuration_calls'] == reads['calibration_calls'] == 0
    assert firmware['current'] is False and firmware['device_motion_firmware'] is None
    assert firmware['command_writes'] == firmware['rpc_calls'] == firmware['lease_requests'] == 0
    assert firmware['lowstate_version']['decoded_version'] is None
    assert firmware['foot_force']['left_load_n'] is firmware['foot_force']['right_load_n'] is firmware['foot_force']['cabinet_load_n'] is None
    assert applicability['current'] is False
    assert applicability['command_writes'] == applicability['lease_requests'] == 0
    assert all(c['load_n'] is c['contact_verified'] is None for c in applicability['contacts'])
    assert all(c['applicable_to_current_pose'] is None and c['execution_allowed'] is False for c in applicability['capabilities'])
    assert all(v is None for k,v in applicability['slow_control'].items() if k != 'meaning')
    assert applicability['ota_history']['device_motion_firmware'] is None
    assert {(r['service'], r['api_id']) for r in reads['readings']} == {
        ('sport',1), ('motion_switcher',1), ('arm',1), ('sport',7003), ('sport',7005), ('arm',7107)}
    return dict(**audit, human_observation=human, official_reads=reads, paired_observation=observation,
        manual_review=manual, firmware_review=firmware, applicability_review=applicability,
        conditional_plan=applicability['conditional_plan'])


def github_audit():
    """Public discussion is provenance, never per-device calibration or an RPC allowlist."""
    evidence = json.loads((ROOT/'g1_console/assets/g1_github_parameter_audit.json').read_text())
    source = evidence['snapshot_source']
    pinned(source['path'], source['sha256'])
    snapshot = json.loads((ROOT/source['path']).read_text())
    assert evidence['search_query_count'] == len(snapshot['queries'])
    assert evidence['search_result_count'] == len(snapshot['issues'])
    assert evidence['full_comment_thread_count'] == len(snapshot['comment_threads'])
    assert evidence['comment_count'] == sum(t['comments_retrieved'] for t in snapshot['comment_threads'])
    assert evidence['repositories'] == snapshot['repositories']
    assert all(q['all_pages_retrieved'] and sum(p['retrieved'] for p in q['pages']) == q['total_count'] for q in snapshot['queries'])
    assert evidence['motor_configuration_read_supported'] is evidence['calibration_read_supported'] is None
    assert evidence['motor_configuration_values_obtained'] == evidence['robot_rpc_calls'] == evidence['command_writes'] == 0
    assert evidence['confirmed_read_endpoints'] == []
    assert all(f['confirmed_g1_read_protocol'] is False for f in evidence['findings'])
    return evidence


def interface_audit():
    """Official protocol review and historical RPC metadata are not motor settings."""
    evidence = json.loads((ROOT/'g1_console/assets/g1_interface_parameter_audit.json').read_text())
    files = evidence['source_files'] + [evidence['query_source']] + [r['inventory'] for r in evidence['repositories']]
    for source in files:
        pinned(source['path'], source['sha256'])
    query = json.loads((ROOT/evidence['query_source']['path']).read_text())
    assert query['command_writes'] == query['configuration_calls'] == query['calibration_calls'] == query['lease_requests'] == 0
    assert query['motor_configuration_values_obtained'] == evidence['motor_configuration_values_obtained'] == 0
    assert all(row['api_id'] == 1 for row in query['versions'])
    assert query['robot_state']['services']['api_id'] == 1003 and query['robot_state']['packages']['api_id'] == 1006
    return dict(**evidence, rpc_metadata=query)


def board_audit(motors):
    """Historical device evidence never becomes a live reading or a motor configuration."""
    evidence = json.loads((ROOT/'g1_console/assets/g1_board_parameter_audit.json').read_text())
    for key in ('passive_read', 'installed_model'):
        source = evidence[key]
        pinned(source['path'], source['sha256'])
    raw = json.loads((ROOT/evidence['passive_read']['path']).read_text())
    assert raw['command_writes'] == raw['rpc_calls'] == raw['motor_configuration_values_obtained'] == 0
    discovery = raw['dds_discovery']
    topics = []
    for name, reading in raw['passive_samples'].items():
        row = dict(topic=name, samples=reading['samples'], status=reading['status'],
                   received_at_utc=reading.get('received_at_utc'), crc_valid=reading.get('crc_valid'),
                   publisher_device_verified=False, raw_fields=reading.get('last'))
        if name == 'rt/lowcmd':
            commands = reading.get('last', {}).get('motors', [])
            row['raw_fields'] = dict(protocol_motor_slots=len(commands),
                last_payload_all_kp_kd_zero=bool(commands) and all(m['kp']==m['kd']==0 for m in commands))
            row['meaning'] = '被动读到的最后一帧命令载荷；发布者物理身份、驱动是否采纳和当前控制器均未核定，不能填实机 Kp/Kd'
        else:
            row['meaning'] = '带时间的原始字段；不推定数组通道、单位、位表或固件版本，不作为当前状态'
        topics.append(row)
    model = ET.parse(ROOT/evidence['installed_model']['path']).getroot()
    assert model.get('model') == 'g1_23dof_rev_1_0'
    by_name = {m['name']:m for m in motors}
    joints = []
    for j in model.findall('.//worldbody//joint'):
        if j.get('name') not in by_name:
            continue
        m = by_name[j.get('name')]
        joints.append(dict(index=m['index'],name=m['name'],
            position_rad=[float(v) for v in j.get('range').split()],
            actuator_force_range_nm=[float(v) for v in j.get('actuatorfrcrange').split()]))
    assert len(joints) == 23
    differences = [dict(index=j['index'], name=j['name'], installed_model_force_nm=j['actuator_force_range_nm'],
                        documented_urdf_effort_nm=by_name[j['name']]['documented_variant']['model']['effort_nm'])
                   for j in joints if max(abs(v) for v in j['actuator_force_range_nm']) !=
                   by_name[j['name']]['documented_variant']['model']['effort_nm']]
    return dict(current=False, checked_at_utc=raw['checked_at_utc'],completed_at_utc=raw['completed_at_utc'],
        meaning='历史只读核查，重启后仍保留；实时状态必须另读。未取得新的逐电机配置值',
        sources=evidence, pc1_ssh=raw['pc1_ssh'],orin_files=raw['orin_files'],
        command_writes=0,rpc_calls=0,motor_configuration_values_obtained=0,physical_board_count=None,
        discovery_counts={key:len(discovery[key]) for key in ('participants','publications','subscriptions')},
        discovery_limitations=discovery['limitations'], topic_readings=topics,
        unqueried_services=sorted({r['topic'].split('/')[2] for r in discovery['subscriptions']
            if r['topic'].startswith('rt/api/') and r['topic'].endswith('/request') and
            r['topic'].split('/')[2] in ('config','basic_clearoip','basic_demarcate','basic_softlimit','basic_taumax','waist_pitch_demarcate','robot_type_service')}),
        service_limitation='只发现服务声明，未取得匹配的公开只读协议；未试发配置、标定或限值请求',
        installed_model=dict(name=model.get('model'),joints=joints,differences=differences,
            active_controller_use_verified=False,
            meaning='Orin 安装目录中的模型参考；文件存在不证明当前控制器加载，也不是实机标定或固件限值'))


def documented_hardware(sources):
    """Read pinned primary sources as data; do not run any vendor control code."""
    additional = json.loads((ROOT/'g1_console/assets/official_motor_sources/sources.json').read_text())
    for key, source in additional.items():
        pinned(source['path'], source['sha256'])
    sources.update(additional)
    text = (ROOT/sources['variants']['path']).read_text()
    row = next(line for line in text.splitlines() if re.match(r'\|\s*`g1_23dof_rev_1_0`\s*\|', line))
    cells = [c.strip() for c in row.strip('|').split('|')]
    mode = int(cells[1]); ratios = [float(v) for v in re.findall(r'[\d.]+', cells[2])]
    assert mode == 4 and len(ratios) == 2 and cells[6:9] == ['6*2','1','5*2']
    variant = ET.parse(ROOT/sources['variant_urdf']['path']).getroot()
    joints = {j.get('name'):j for j in variant.findall('joint') if j.get('type') != 'fixed'}
    assert len(joints) == 23
    cpp = (ROOT/sources['terminations']['path']).read_text()
    channels = []
    for component, label in [('casing','外壳'),('winding','绕组')]:
        match = re.search(r'motor_'+component+r'_overheat\([^)]*limit_temp\s*=\s*([\d.]+)\).*?temperature\(\)\[(\d)\]', cpp, re.S)
        assert match, component
        channels.append(dict(index=int(match[2]), component=component, label=label, source='terminations',
                             sdk_helper_default_c=float(match[1])))
    return dict(mode_machine=mode, name=cells[0].strip('`'), dof=23, source='variants', model_source='variant_urdf',
                hip_gear_ratios=dict(pitch=ratios[0], roll=ratios[1]),
                model_total_mass_kg=sum(float(l.find('inertial/mass').get('value')) for l in variant.findall('link') if l.find('inertial/mass') is not None),
                meaning='官方机型表定义；仅当新鲜机型代码和 23 关节配置匹配时适用，不代表逐电机序号或标定已核验'), joints, sorted(channels,key=lambda c:c['index'])


def motion_control_review():
    review = json.loads((ROOT/'g1_console/assets/g1_motion_control_review.json').read_text())
    for source in review['sources']:
        pinned(source['path'], source['sha256'])
    assert review['current'] is review['execution_allowed'] is False
    assert review['device_motion_firmware'] is None
    assert review['motion_writes'] == review['motor_configuration_values_obtained'] == 0
    assert all(row['device_applicable'] is None and not row['execution_allowed'] for row in review['modes'])
    assert any(s['path'] == review['app_evidence_source'] for s in review['sources'])
    app = json.loads((ROOT/review['app_evidence_source']).read_text())
    assert app['current'] is app['execution_allowed'] is False
    assert app['captured_at_utc'] is app['device']['motion_firmware'] is None
    assert app['motion_writes'] == app['motor_configuration_values_obtained'] == 0
    review['app_observation'] = app
    assert any(s['path'] == review['service_evidence_source'] for s in review['sources'])
    review['service_readiness'] = service_readiness(json.loads((ROOT/review['service_evidence_source']).read_text()))
    for name in ('humanoid_evidence_source', 'humanoid_files_source'):
        assert any(s['path'] == review[name] for s in review['sources'])
    review['humanoid_review'] = humanoid_review(
        json.loads((ROOT/review['humanoid_evidence_source']).read_text()),
        json.loads((ROOT/review['humanoid_files_source']).read_text()))
    assert any(s['path'] == review['publisher_evidence_source'] for s in review['sources'])
    review['publisher_review'] = publisher_review(
        json.loads((ROOT/review['publisher_evidence_source']).read_text()))
    for name in ('official_manual_source', 'motor_test_evidence_source'):
        assert any(s['path'] == review[name] for s in review['sources'])
    manual = json.loads((ROOT/review['official_manual_source']).read_text())
    assert manual['branch_selected'] is None
    assert any(s['path'] == manual['pdf_path'] and s['sha256'] == manual['pdf_sha256'] for s in review['sources'])
    review['official_manual'] = manual
    result = json.loads((ROOT/review['motor_test_evidence_source']).read_text())
    assert result['current'] is result['execution_allowed'] is False
    assert result['motion_writes'] == 1 and result['sdk_code'] == 3104
    assert result['observed_attempt']['status'] == 'outcome_unknown'
    assert result['support_verified'] is result['motion_authority_verified'] is False
    assert result['device_motion_firmware'] is result['exact_7301_predicate'] is None
    for source in result['files']:
        pinned(source['path'], source['sha256'])
    review['motor_test_contract']['device_result'] = result
    assert any(s['path'] == review['reboot_readonly_audit_source'] for s in review['sources'])
    audit = json.loads((ROOT/review['reboot_readonly_audit_source']).read_text())
    assert audit['current'] is audit['execution_allowed'] is audit['motion_authority_verified'] is False
    assert audit['device_motion_firmware'] is audit['exact_7301_predicate'] is None
    assert audit['command_writes'] == audit['lease_requests'] == 0
    assert audit['official_processes_unchanged'] and audit['official_processes_before'] == audit['official_processes_after']
    assert all(row['raw'] is None for row in audit['reads'] if row['code'] != 0)
    review['reboot_readonly_audit'] = audit
    assert any(s['path'] == review['installed_sdk_audit_source'] for s in review['sources'])
    installed = json.loads((ROOT/review['installed_sdk_audit_source']).read_text())
    assert installed['current'] is installed['execution_allowed'] is installed['motion_authority_verified'] is False
    assert installed['device_motion_firmware'] is installed['exact_7301_predicate'] is None
    assert installed['bilateral_foot_load_n'] is installed['external_support_load_n'] is None
    assert installed['constructor_rpc_requests'] == installed['init_rpc_requests'] == 0
    assert installed['command_writes'] == installed['lease_requests'] == 0
    assert installed['before'] == installed['after'] == dict(fsm=0, owner=None, armed=False)
    assert {r['api_id'] for r in installed['sdk_getters']} == {1, *range(7001, 7007)}
    assert len(installed['sdk_getters']) == 7
    for row in installed['sdk_getters']:
        assert row['replies'] and all(r['code'] == row['sdk_code'] for r in row['replies'])
        if row['api_id'] in range(7003, 7007):
            assert row['sdk_code'] == 7301 and row['sdk_value'] is None
            assert all(r['data'] is None for r in row['replies'])
    for source in installed['sources']:
        pinned(source['local_source'], source['sha256'])
    review['installed_sdk_audit'] = installed
    return review


def build():
    lock = json.loads((ROOT/'unitree-deps.lock.json').read_text())['dependencies']
    sdk = 'externals/unitree_sdk2_python/'
    model = 'externals/unitree_rl_gym/resources/robots/g1_description/'
    paths = dict(feedback=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorState_.py',
                 command=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorCmd_.py',
                 lowstate=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_LowState_.py',
                 lowcmd=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_LowCmd_.py',
                 mainboard=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_MainBoardState_.py',
                 bms=sdk+'unitree_sdk2py/idl/unitree_hg/msg/dds_/_BmsState_.py',
                 example=sdk+'example/g1/low_level/g1_low_level_example.py',
                 loco=sdk+'unitree_sdk2py/g1/loco/g1_loco_client.py',
                 urdf=model+'g1_23dof.urdf', mujoco=model+'g1_23dof.xml')
    sources = {}
    for key, path in paths.items():
        repo = 'unitree_sdk2_python' if path.startswith(sdk) else 'unitree_rl_gym'
        prefix = sdk if repo == 'unitree_sdk2_python' else 'externals/unitree_rl_gym/'
        commit = lock[repo]['commit']
        sources[key] = dict(path=path, commit=commit, sha256=hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),
            url=f'https://github.com/unitreerobotics/{repo}/blob/{commit}/{path.removeprefix(prefix)}')
    hardware, variant_joints, channels = documented_hardware(sources)
    example = ast.parse((ROOT/paths['example']).read_text())
    gains = {n.targets[0].id: ast.literal_eval(n.value) for n in example.body if isinstance(n, ast.Assign)
             and isinstance(n.targets[0], ast.Name) and n.targets[0].id in ('Kp', 'Kd')}
    urdf = ET.parse(ROOT/paths['urdf']).getroot()
    uj = {j.get('name'): j for j in urdf.findall('joint')}
    links = {l.get('name'): l for l in urdf.findall('link')}
    body = json.loads((ROOT/'g1_console/assets/g1_23.json').read_text())
    motors = []
    for spec in body['joints']:
        j = uj[spec['name']]; limit = j.find('limit')
        link = links[j.find('child').get('link')]; inertial = link.find('inertial')
        dynamics = j.find('dynamics')
        vl = variant_joints[spec['name']].find('limit')
        gear = next((ratio for axis,ratio in hardware['hip_gear_ratios'].items()
                     if spec['name'] in (f'left_hip_{axis}_joint',f'right_hip_{axis}_joint')), None)
        motors.append(dict(index=spec['index'], name=spec['name'], label=spec['label'],
            documented_variant=dict(mode_machine=hardware['mode_machine'], gear_ratio=gear,
                gear_source='variants' if gear is not None else None,
                model=dict(position_rad=[float(vl.get('lower')),float(vl.get('upper'))],
                           velocity_rad_s=float(vl.get('velocity')), effort_nm=float(vl.get('effort')),source='variant_urdf'),
                serial_verified=False),
            model=dict(position_rad=[float(limit.get('lower')), float(limit.get('upper'))],
                       velocity_rad_s=float(limit.get('velocity')), effort_nm=float(limit.get('effort')),
                       actuator_force_nm=spec['torque_range'], axis=spec['axis'],
                       origin=dict(j.find('origin').attrib), parent=j.find('parent').get('link'), child=link.get('name'),
                       link_mass_kg=float(inertial.find('mass').get('value')),
                       inertia_kg_m2={k:float(v) for k,v in inertial.find('inertia').attrib.items()},
                       dynamics={k:float(v) for k,v in dynamics.attrib.items()} if dynamics is not None else None),
            example_only=dict(kp=gains['Kp'][spec['index']], kd=gains['Kd'][spec['index']],
                              source='example', meaning='官方低层示例设定；不是出厂值或当前运动控制器参数；未执行示例'),
            actual_config=dict(kp=None, kd=None, continuous_torque_nm=None, peak_torque_nm=None,
                current_limit_a=None, gear_ratio=None, zero_offset_rad=None, motor_model=None,
                serial_number=None, firmware=None, thermal_limits_c=None)))
    feedback = [
        ('mode','uint8','raw','原始模式码；官方示例命令设置 1，不据此推定当前闭环增益或健康'),
        ('q','float32','rad','角度；并联关节需结合 LowState.mode_pr 的 PR/AB 坐标'),
        ('dq','float32','rad/s','角速度'), ('ddq','float32','rad/s²','角加速度；零值不证明固件支持有效测量'),
        ('tau_est','float32','N·m','估算力矩；不是独立测力传感器或连续额定值'),
        ('temperature','int16[2]','°C','官方 G1 SDK 定义：[0] 外壳、[1] 绕组；SDK 辅助函数默认值不是本机固件热保护阈值'),
        ('vol','float32','V','电机电压；不能换算剩余电量'),
        ('sensor','uint32[2]','raw','原始传感器字段；未取得本机型解码文档'),
        ('motorstate','uint32','raw','原始状态位；未取得本固件位表，0 也不证明全部健康'),
        ('reserve','uint32[4]','raw','保留字段；不解释为有效物理参数')]
    commands = [('mode','uint8','raw'),('q','float32','rad'),('dq','float32','rad/s'),
                ('tau','float32','N·m'),('kp','float32','N·m/rad'),('kd','float32','N·m·s/rad'),('reserve','uint32','raw')]
    review = motion_control_review()
    app = review['app_observation']
    data = dict(schema_version=27, robot='G1 EDU 23 DOF', hands=False, computer='Jetson Orin',
        configuration_source='用户确认的 G1 EDU / 23 自由度 / 无手 / Orin；并非仅凭固件代码推定',
        identity=dict(serial_number=None, firmware_version=None,
            app_reported_model=app['device']['model'],
            app_device_software_version=app['device']['software_version'],
            app_device_hardware_version=app['device']['hardware_version'],
            app_evidence_source=review['app_evidence_source'], app_captured_at_utc=None,
            authority='App 截图字段；设备绑定由用户报告，非运动固件或自由度自动识别'),
        model_total_mass_kg=sum(float(l.find('inertial/mass').get('value')) for l in links.values() if l.find('inertial/mass') is not None),
        persistence='项目静态基础档案；观察时直接加载，不由语言模型重复推断',
        authority='模型和示例不等于本机标定；未知值必须为空；仅用于只读解释，不授权动作', sources=sources,
        documented_hardware=hardware, temperature_channels=channels, state_safety=state_safety_knowledge(sources),
        safety_evidence_audit=safety_evidence_audit(),
        standing_knowledge=standing_knowledge(),
        motion_control_review=review,
        thermal_authority='温度通道采用官方 G1 SDK 语义；85/120 °C 仅为可传参覆盖的 SDK 辅助函数默认值，未核定为本机固件保护阈值或安全工作温度',
        device_audit=json.loads((ROOT/'g1_console/assets/g1_device_parameter_audit.json').read_text()),
        board_audit=board_audit(motors),
        interface_audit=interface_audit(),
        github_review=github_audit(),
        service_audit=service_audit(), official_capabilities=capability_knowledge(),
        rejected_inferences=[dict(source='motor_catalog',reason='通用电机表未逐项绑定 mode_machine=4，且髋俯仰减速比与本版本不一致；不据此填写本机电机型号或额定力矩'),
                             dict(source='variant_urdf',reason='对应机型 URDF effort 和速度仍是模型限值，不是实机连续/峰值规格或固件限制'),
                             dict(source='terminations',reason='辅助函数默认阈值不是从实机读取的热保护配置')],
        feedback_fields=[dict(name=n,type=t,unit=u,meaning=m,source='feedback') for n,t,u,m in feedback],
        command_fields=[dict(name=n,type=t,unit=u,source='command',meaning='命令字段定义；当前值不可由 LowState 读取，本项目不发布') for n,t,u in commands],
        coordinate_modes={'0':'PR：踝俯仰/侧摆关节坐标（官方示例定义）','1':'AB：并联机构 A/B 电机坐标（不能直接比较 PR 关节模型）'},
        fsm_sdk_mapping={'0':'SDK ZeroTorque 对应状态 ID','1':'SDK Damp 对应状态 ID','3':'SDK Sit 对应状态 ID',
                         '500':'SDK Start 对应行走状态 ID','702':'SDK Lie2StandUp 对应状态 ID','706':'SDK 蹲站转换对应状态 ID'},
        unknowns=['本机出厂及当前控制器 Kp/Kd','本机电机型号、序列号和驱动固件','连续/峰值力矩与允许时长、转矩转速曲线',
                  '电流上限、转矩常数、其余关节减速比及效率；髋俯仰/侧摆已取得机型文档值，未逐电机标定核验','零位偏移、编码器标定、摩擦/回差','本机固件热阈值和降额规则；温度通道已取得官方 SDK 定义',
                  'motorstate 位表与 sensor 字段语义','足底接触/负载、全身支撑状态；BMS 原始消息已只读接入，单位/来源/故障语义仍待核验'], motors=motors)
    data['foundation_id'] = hashlib.sha256(json.dumps(data, ensure_ascii=False, sort_keys=True).encode()).hexdigest()
    return data


if __name__ == '__main__':
    data = build()
    target = ROOT/'g1_console/assets/g1_motor_knowledge.json'
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print(f'{len(data["motors"])} motors; {len(data["feedback_fields"])} feedback and {len(data["command_fields"])} command fields; {target}')
