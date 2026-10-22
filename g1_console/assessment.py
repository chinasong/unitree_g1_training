"""Deterministic, read-only interpretation. Matching a condition never grants motion."""
import math
from .body import JOINTS, KNOWLEDGE


def condition(name, status, detail, source):
    return dict(name=name, status=status, detail=detail, source=source)


def combined(checks):
    return 'violated' if any(c['status'] == 'violated' for c in checks) else 'unknown' if not checks or any(c['status'] == 'unknown' for c in checks) else 'matched'


def assess(telemetry, fresh, fsm_age_ms, capabilities, firmware, read_only, history=()):
    measured = {j['index']:j for j in telemetry['joints']}
    pr = telemetry.get('mode_pr')
    fsm = telemetry.get('fsm')
    fsm_fresh = fresh and fsm_age_ms is not None and fsm_age_ms < 1500
    joints = []
    for spec in JOINTS:
        data = measured.get(spec['index'], {})
        q = data.get('q'); lo, hi = spec['range']
        comparable = fresh and q is not None and (spec['index'] not in (4,5,10,11) or pr == 0)
        delta = max(lo-q, q-hi, 0) if comparable else None
        joints.append(dict(index=spec['index'], label=spec['label'], position_rad=q,
            model_range_rad=[lo,hi], comparable=comparable, deviation_rad=delta,
            status='unknown' if not comparable else 'violated' if delta > 0 else 'matched',
            detail='反馈已过期或缺失' if not fresh or q is None else
                   'PR/AB 坐标未确认，不能直接比较模型' if not comparable else
                   '略超模型边界；需核对版本与标定，不能据此判定故障' if 0 < delta <= .01 else
                   '超出参考模型范围；需核对实机版本、坐标及标定' if delta > .01 else '处于参考模型角度范围内'))
    speeds = [abs(j['dq']) for j in measured.values() if j.get('dq') is not None] if fresh else []
    temps = [v for j in measured.values() for v in j.get('temperature') or []] if fresh else []
    voltages = [j['voltage'] for j in measured.values() if j.get('voltage') is not None] if fresh else []
    imu = telemetry.get('imu') if fresh else None
    knees = [measured.get(i, {}).get('q') for i in (3,9)] if fresh else []
    summary = '身体反馈未就绪；所有当前条件待核实'
    if fresh:
        summary = f'FSM {fsm}：{KNOWLEDGE["fsm_sdk_mapping"].get(str(fsm), "语义未核定")}' if fsm_fresh else 'FSM 回读过期；控制状态未知'
        if len(knees) == 2 and all(v is not None for v in knees):
            summary += f'；双膝角度 {math.degrees(knees[0]):.1f}° / {math.degrees(knees[1]):.1f}°'
        summary += '；足底接触及主动支撑未确认'
    live_checks = [condition('身体反馈', 'matched' if fresh else 'unknown', '23 关节反馈在线' if fresh else '无有效的新鲜反馈', 'rt/lowstate + 本项目时效检查'),
        condition('行走状态条件', 'unknown' if not fsm_fresh else 'matched' if fsm == 500 else 'violated',
                  f'当前 FSM={fsm}；本项目速度控制要求 FSM=500' if fsm_fresh else 'FSM 回读缺失或已超过 1.5 秒', '本项目运动门槛；官方 SDK Start→500'),
        condition('稳定站立 / 不摔倒', 'unknown', '单帧关节与 IMU 不能确认接触、支撑和稳定性', '未接入足底接触、载荷与全身位姿'),
        condition('电机健康结论', 'unknown', '保留 mode / motorstate 原始值，尚无本固件故障位表与热阈值', 'HG MotorState IDL 仅提供字段类型')]
    def evaluate(row):
        checks = []
        for key, actual, valid in (('fsm', fsm, fsm_fresh), ('mode_machine', telemetry.get('mode_machine'), fresh)):
            if key in row:
                values = row[key]
                checks.append(condition(key, 'unknown' if not valid or actual is None else 'matched' if actual in values else 'violated',
                    f'当前 {actual}；固件声明 {values}', '实机 GetActionList'))
        if not checks:
            checks.append(condition('附加条件', 'unknown', '固件未声明条件；不能理解为无条件可执行', '实机 GetActionList'))
        return dict(status=combined(checks), conditions=checks, execution_allowed=False,
                    note='只比较已声明条件；不证明动作安全、支持全部接口或现场适用')
    firmware_results = [dict(row, compatibility=evaluate(row)) for row in firmware]
    by_id = {r['id']:r['compatibility'] for r in firmware_results if r['id'] is not None}
    caps = []
    for cap in capabilities:
        checks = []
        if cap['risk'] == 'read':
            checks.append(condition('读取连接', 'matched' if fresh else 'unknown', '只读查询入口已接通；实际调用仍可能返回错误', '本项目查询白名单'))
        elif cap.get('action_id') is not None and cap['action_id'] in by_id:
            checks.extend(by_id[cap['action_id']]['conditions'])
        elif cap['id'] == 'velocity':
            checks.append(live_checks[1])
        else:
            checks.append(condition('执行前提', 'unknown', '尚无经核定的完整前提；SDK 定义不证明当前可执行', '锁定版本 SDK'))
        caps.append(dict(id=cap['id'], status=combined(checks), conditions=checks, execution_allowed=False,
                         policy='只读锁定' if read_only and cap['risk'] != 'read' else '条件分析不授予执行权限'))
    window = list(history) if fresh else []
    hardware = KNOWLEDGE['documented_hardware']
    configured = len(measured) == hardware['dof'] and set(measured) == {j['index'] for j in JOINTS}
    variant_status = 'unknown' if not fresh or telemetry.get('mode_machine') is None else 'matched' if configured and telemetry['mode_machine'] == hardware['mode_machine'] else 'violated'
    variant_detail = (f'官方表对应 {hardware["name"]}；文档髋俯仰/侧摆减速比 {hardware["hip_gear_ratios"]["pitch"]} / {hardware["hip_gear_ratios"]["roll"]}；未逐电机标定核验'
                      if variant_status == 'matched' else '机型反馈或关节配置不匹配；本档案的 mode 4 文档参数不适用于当前设备' if variant_status == 'violated' else '等待新鲜机型代码和 23 关节反馈，文档适用性未知')
    return dict(schema_version=1, summary=summary, current=fresh, fsm_fresh=fsm_fresh,
        fsm_label=KNOWLEDGE['fsm_sdk_mapping'].get(str(fsm)) if fsm_fresh else None,
        coordinate_mode=KNOWLEDGE['coordinate_modes'].get(str(pr), '未确认'),
        mode_machine_note=variant_detail,
        hardware_variant=condition('官方机型文档适用性',variant_status,variant_detail,KNOWLEDGE['sources']['variants']['url']),
        movement=dict(max_abs_dq_rad_s=max(speeds) if speeds else None,
            description='本帧低速；不证明稳定站立' if speeds and max(speeds)<.05 else '当前关节速度见反馈' if speeds else '未知',
            descriptive_threshold_rad_s=.05, threshold_source='本项目显示规则，非厂商安全阈值',
            window_samples=len(window), window_s=round(window[-1]['at']-window[0]['at'],2) if len(window)>1 else 0,
            window_max_abs_dq_rad_s=max(r['max_dq'] for r in window) if window else None),
        imu_rpy_deg=[math.degrees(v) for v in imu['rpy']] if imu else None,
        temperature_range_c=[min(temps),max(temps)] if temps else None,
        voltage_range_v=[min(voltages),max(voltages)] if voltages else None,
        checks=live_checks, joint_limits=joints, capabilities=caps, firmware_actions=firmware_results,
        unknowns=KNOWLEDGE['unknowns'], execution_allowed=False)
