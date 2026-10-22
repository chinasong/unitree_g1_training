"""Extract every public business method of pinned G1 clients as inert source data.

No vendor imports. Python signatures/defaults and C++ overloads stay separate.
The catalog never registers an RPC, enables a console command or fills motor values.
"""
import ast
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SDK = 'externals/unitree_sdk2_python/'


def digest(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def read_source(source):
    assert digest(source['path']) == source['sha256'], source['path']
    return (ROOT/source['path']).read_text()


def constants(text, language):
    if language == 'Python':
        values = {}
        for n in ast.parse(text).body:
            if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name):
                try:
                    values[n.targets[0].id] = ast.literal_eval(n.value)
                except ValueError:
                    pass
        return values
    return {k:json.loads(v) for k,v in re.findall(r'const\s+(?:std::string|int32_t)\s+(\w+)\s*=\s*("[^"]*"|\d+)\s*;', text)}


def cpp_methods(text):
    for match in re.finditer(r'\bint32_t\s+(\w+)\s*\(([^)]*)\)\s*\{', text):
        name = match[1]
        if name.startswith('_'):
            continue  # generic multiplexed transport is not a capability
        depth, end = 1, match.end()
        while depth and end < len(text):
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        assert depth == 0, name
        yield name, match[2].strip(), text[match.start():end], text[:match.start()].count('\n')+1


def semantics(group, name, language):
    notes = []
    payload = None
    if name.startswith('Get'):
        notes.append('读取定义；成功与否及有效值以带时间的实机返回为准。')
    if group == 'loco':
        scalar = {'SetFsmId':'fsm_id','SetBalanceMode':'balance_mode','SetSwingHeight':'swing_height',
                  'SetStandHeight':'stand_height','SetTaskId':'task_id','SetSpeedMode':'speed_mode'}
        if name in scalar:
            payload = {'data':scalar[name]}
        fsm = {'Damp':1,'Start':500,'Sit':3,'ZeroTorque':0,'Squat2StandUp':706,'StandUp2Squat':706,'Lie2StandUp':702,'Squat':2,'StandUp':4}
        if name in fsm:
            payload = {'SetFsmId':fsm[name]}
            notes.append('SDK 别名映射；不同版本的蹲站状态 ID 不可互换，不保证当前固件支持。')
        if name in ('SetVelocity','Move','StopMove'):
            payload = {'velocity':['vx','vy','omega / vyaw'],'duration':'SetVelocity 默认 1.0；Move 连续模式 864000，非连续 1；StopMove 三轴为 0'}
            notes.append('SDK 连续时长不是本项目运动许可；实际控制台限速与 250 ms 指令时效另行约束。')
        if name in ('HighStand','LowStand'):
            payload = {'SetStandHeight':4294967295 if name=='HighStand' else 0}
            notes.append('uint32 最大/最小值为 SDK 高低站姿哨兵，不是物理高度。')
        if name in ('GetStandHeight','GetSwingHeight','SetStandHeight','SetSwingHeight'):
            notes.append('该头文件/实现未给出物理单位和硬件有效范围，保留未知。')
        if name == 'SetTaskId':
            notes.append('Python 注解为 float，C++ 为 int；不能据此推定任意任务 ID 有效。')
        if name == 'SwitchToUserCtrl':
            payload = {'data':False} if language=='Python' else None
            notes.append('切换控制器会改变状态；只保存定义，不调用。')
        if name == 'SwitchToInternalCtrl':
            payload = {'data':{'LAST':0,'PASSIVE':1,'WALKRUN':2}}
        if name == 'BalanceStand':
            payload = {'SetBalanceMode':'balance_mode 参数' if language=='Python' else 0}
        if name == 'ContinuousGait':
            payload = {'SetBalanceMode':'flag ? 1 : 0'}
        if name == 'SwitchMoveMode':
            notes.append('仅修改 C++ 客户端 continous_move_ 本地变量，不发送 RPC。')
        if name == 'WaveHand':
            payload = {'SetTaskId':'turn_flag ? 1 : 0'}
        if name == 'ShakeHand':
            payload = {'SetTaskId':'stage=0 -> 2；stage=1 -> 3；其他值切换客户端本地状态'}
            notes.append('默认 stage=-1 依赖客户端历史，不代表已经收到机器人握手反馈。')
        if name == 'GetPhase':
            notes.append('官方已标记 deprecated，本轮不查询。')
        if name == 'GetMimicMotion':
            payload = {'fsm_id':550,'api_id':1}
            notes.append('C++ 文档要求稳定状态；尚未确认该前提，不调用 API 7100。响应 motion 按键给出 name/duration，不能编造本机动作列表。')
    elif group == 'arm':
        if name == 'ExecuteAction':
            payload = {'data':'action_id'} if language=='Python' else {'action_id / action_name':'见对应重载源码，禁止混用 Python 与 C++ 载荷'}
            notes.append('Python ID 载荷为 data；C++ 为 action_id，自定义名称为 action_name。ID 99 是释放动作。官方提示部分动作可能导致跌倒，目录声明不能证明安全。')
        if name == 'StopCustomAction':
            notes.append('C++ 专有；会改变执行状态，本轮不调用。')
        if name == 'GetActionList':
            notes.append('实际返回含 ID、名称及可选 FSM / mode_machine / time；无 ID 条目不得猜测调用编号。')
    elif group == 'audio':
        payload = {'TtsMaker':{'index':'客户端计数','text':'text','speaker_id':'speaker_id'},
                   'GetVolume':{},'SetVolume':{'volume':'volume'},'LedControl':{'R':'R','G':'G','B':'B'},
                   'PlayStream':{'app_name':'app_name','stream_id':'stream_id','binary':'pcm_data'},
                   'PlayStop':{'app_name':'app_name'}}.get(name)
        if name == 'TtsMaker':
            notes.append('speaker_id 可用范围未由该源码声明；Python tts_index += tts_index 从 0 不增长，C++ 使用后置自增，保留版本差异。')
        if name == 'PlayStop':
            notes.append('两种 SDK 均丢弃底层返回码并固定返回 0；不能把它当作设备成功应答。')
        if name in ('SetVolume','LedControl'):
            notes.append('C++ uint8_t 只给出传输类型宽度，不代表音量或亮度的实机有效范围。')
        if name == 'PlayStream':
            notes.append('此客户端源码未声明采样率、通道数及长度上限；控制台音频格式/长度限制属于项目策略。')
    elif group == 'video':
        notes.append('复用官方 go2 命名空间 VideoClient 的 videohub 读取接口；不是已取得 G1 相机标定或实时图像的证明。')
        payload = []
    elif group == 'agv':
        notes.append('仅适用 AGV 底盘；本机 EDU 23 DOF 未确认安装，不能当作当前双足机器人的能力或限值。')
        if name == 'Move':
            payload = {'vx':'vx','vy':'vy（AGV 忽略）','vyaw':'vyaw'}
            notes.append('AGV 文档专用：vx ±1.5 m/s，vyaw ±0.6 rad/s；不可套用于双足 G1。')
        else:
            payload = {'data':'vz'}
            notes.append('AGV 专用 vz [-1,1]，column_velocity = vz × 76.5 mm/s；不是 G1 站高参数。')
    return payload, notes


def build():
    sources = json.loads((ROOT/'g1_console/assets/capability_sources/sources.json').read_text())
    commit = json.loads((ROOT/'unitree-deps.lock.json').read_text())['dependencies']['unitree_sdk2_python']['commit']
    for group, folder, stem in [('loco','g1/loco','g1_loco'),('arm','g1/arm','g1_arm_action'),('audio','g1/audio','g1_audio'),('video','go2/video','video')]:
        for kind in ('client','api'):
            rel = f'unitree_sdk2py/{folder}/{stem}_{kind}.py'
            path = SDK+rel
            sources[f'py_{group}_{kind}'] = dict(path=path,sha256=digest(path),commit=commit,
                repository_path=rel,url=f'https://github.com/unitreerobotics/unitree_sdk2_python/blob/{commit}/{rel}')
    methods, protocols = [], []
    for key, source in sources.items():
        text = read_source(source)
        if not key.endswith('_client'):
            continue
        lang, group, _ = key.split('_'); language = 'Python' if lang=='py' else 'C++'
        api = constants(read_source(sources[f'{lang}_{group}_api']), language)
        service = next(v for k,v in api.items() if k.endswith('SERVICE_NAME'))
        version = next(v for k,v in api.items() if k.endswith('API_VERSION'))
        extracted = []
        if lang == 'py':
            for cls in (n for n in ast.parse(text).body if isinstance(n,ast.ClassDef)):
                for node in (n for n in cls.body if isinstance(n,ast.FunctionDef) and not n.name.startswith('_') and n.name!='Init'):
                    args = node.args.args[1:]; offset=len(args)-len(node.args.defaults)
                    params = [dict(name=a.arg,type=ast.unparse(a.annotation) if a.annotation else '未声明',
                        direction='in',has_default=i>=offset,
                        default=ast.unparse(node.args.defaults[i-offset]) if i>=offset else None,
                        unit=None,hardware_range=None) for i,a in enumerate(args)]
                    extracted.append((node.name,ast.get_source_segment(text,node),node.lineno,params))
        else:
            for name, signature, code, line in cpp_methods(text):
                params=[]
                for arg in filter(None,(a.strip() for a in signature.split(','))):
                    before, sep, default=arg.partition('=')
                    found=re.match(r'(.+?)(\w+)\s*$',before.strip()); assert found, arg
                    typ=found[1].strip(); pname=found[2]
                    # Getter references are output parameters, never sent on the wire.
                    params.append(dict(name=pname,type=typ,direction='out' if name.startswith('Get') and '&' in typ else 'in',
                        has_default=bool(sep),default=default.strip() if sep else None,unit=None,hardware_range=None))
                extracted.append((name,code,line,params))
        for name, code, line, params in extracted:
            payload, notes=semantics(group,name,language)
            ids=sorted({api[c] for c in re.findall(r'\b(?:ROBOT_|VIDEO_)API_ID_\w+',code) if c in api})
            if name=='GetMimicMotion':ids=[api['ROBOT_API_ID_LOCO_FSM_API']]
            methods.append(dict(id=f'{lang}.{group}.{name}.{len([m for m in methods if m["language"]==language and m["group"]==group and m["name"]==name])}',
                name=name,group=group,language=language,service=service,api_version=version,api_ids=ids,
                parameters=params,wire=payload,notes=notes,source=key,line=line,source_excerpt=code,
                kind='read' if name.startswith('Get') else 'client_local' if name=='SwitchMoveMode' else 'state_change',
                support='AGV 专用，当前硬件未确认' if group=='agv' else 'SDK 定义；本机支持程度另看实收返回',
                executed=False,device_parameter_values_verified=False))
        protocols.append(dict(language=language,group=group,service=service,api_version=version,
            source=f'{lang}_{group}_api',constants=api))
    # Expand alias API IDs from the same language/group only. Never blend SDK dialects.
    for _ in range(3):
        for m in methods:
            calls=set(re.findall(r'(?:self\.)?(\w+)\s*\(',m['source_excerpt']))-{m['name']}
            if not m['api_ids'] and m['kind']!='client_local':
                m['api_ids']=sorted({i for peer in methods if peer['language']==m['language'] and peer['group']==m['group'] and peer['name'] in calls for i in peer['api_ids']})
    unsupported=[]
    for p in protocols:
        constants_used={i for m in methods if m['language']==p['language'] and m['group']==p['group'] for i in m['api_ids']}
        unsupported.extend(dict(language=p['language'],group=p['group'],name=k,api_id=v,
            meaning='常量已声明，但此 SDK 无对应公开业务方法；请求/响应模式未在此建立')
            for k,v in p['constants'].items() if 'API_ID_' in k and v not in constants_used)
    arm = ast.parse(read_source(sources['py_arm_client']))
    action_map=next(ast.literal_eval(n.value) for n in arm.body if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='action_map')
    reads_path='g1_console/assets/service_audit_sources/capability_reads_20261021.json'
    reads=json.loads((ROOT/reads_path).read_text())
    assert reads['command_writes']==reads['lease_requests']==reads['configuration_calls']==reads['calibration_calls']==reads['motor_configuration_values_obtained']==0
    assert [(r['service'],r['api_id']) for r in reads['readings']]==[('sport',7001),('sport',7002),('sport',7003),('sport',7004),('sport',7005),('arm',7107),('voice',1005)]
    return dict(schema_version=1,execution_allowed=False,scope='固定提交的 G1 Loco/Arm/Audio 全部公开业务方法、共享 Video 读取，以及单列的 AGV 变体；不包含厂商未公开的 App/私有能力',
        authority='接口签名、默认值及载荷是 SDK 定义；不是本机当前配置、支持承诺或动作许可。未知单位、范围保持空值。',
        sources=sources,methods=methods,protocols=protocols,unwrapped_constants=unsupported,
        sdk_action_map=action_map,
        differences=['Arm ExecuteAction：Python 使用 data；C++ ID 重载使用 action_id，名称重载使用 action_name，不混用载荷',
            'C++ 静态 right kiss 映射为 12；Python 为 13，保持差异，不更改实际动作入口',
            'Python 蹲站转换 706 / 躺站 702；C++ Squat 2 / StandUp 4，不能合并为同一固件语义',
            'Python SetTaskId 注解 float / C++ int；BalanceStand 的 Python 版本需参数，C++ 固定 0',
            'GetMimicMotion / 示教动作名称重载等只见 C++；并未新增执行入口'],
        project_policy=dict(authority='项目输入约束，不是官方硬件限值',source='g1_console/contracts.py + capabilities.py',
            velocity=dict(vx=[-.3,.3],vy=[-.2,.2],omega=[-.4,.4],duration_s=.25),
            stand_height=[0,1],balance_mode=[0,1],speed_mode=[0,1,2],task_id=[0,1,2,3],volume=[0,100],rgb=[0,255],
            pcm='控制台限定 16 kHz / mono / S16LE，单段 ≤48000 字节；未作为厂商音频固件规格'),
        historical_reads=dict(source=dict(path=reads_path,sha256=digest(reads_path)),**reads))


def service_audit():
    path='g1_console/assets/service_audit_sources/services_20261021.json'
    raw=json.loads((ROOT/path).read_text())
    receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/receipt_20261021.json').read_text())
    for source in receipt['sources']+receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    assert raw['current'] is False
    assert raw['command_writes']==raw['rpc_calls']==raw['configuration_calls']==raw['calibration_calls']==raw['motor_configuration_values_obtained']==0
    analysis_path='g1_console/assets/service_audit_sources/configuration_analysis_20261021.json'
    analysis_receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/configuration_receipt_20261021.json').read_text())
    for source in analysis_receipt['sources']+analysis_receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    analysis=json.loads((ROOT/analysis_path).read_text())
    assert analysis['current'] is False and analysis['offline_only'] is True
    assert all(analysis[k]==0 for k in ('command_writes','rpc_calls','configuration_calls','calibration_calls','motor_configuration_values_obtained'))
    assert analysis['master']['payload_decoded'] is False
    assert analysis['master']['motor_configuration_present'] is None
    assert all(not p[k] for p in analysis['megaflow']['declared_plugins'] for k in ('installed_verified','loaded_verified','execution_allowed'))
    original={f['sha256'] for f in raw['files'] if f['format']=='opaque'}
    assert original=={f['sha256'] for f in analysis['files'] if f['name'] in ('master_service.json','config.bin')}
    fmx_path='g1_console/assets/service_audit_sources/fmx_analysis_20261021.json'
    fmx_receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/fmx_receipt_20261021.json').read_text())
    for source in fmx_receipt['sources']+fmx_receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    fmx=json.loads((ROOT/fmx_path).read_text())
    assert fmx['current'] is False and fmx['offline_only'] is True
    assert fmx['payload_decoded'] is False and fmx['motor_configuration_present'] is None
    assert all(fmx[k]==0 for k in ('motor_configuration_values_obtained','command_writes','rpc_calls','configuration_calls','calibration_calls','vendor_program_executions'))
    expected={f['name']:f['sha256'] for f in analysis['files']}
    assert all(expected[f['name']]==f['sha256'] for f in fmx['files'])
    loader_path='g1_console/assets/service_audit_sources/fmx_loader_review_20261021.json'
    loader_receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/fmx_loader_receipt_20261021.json').read_text())
    for source in loader_receipt['sources']+loader_receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    loader=json.loads((ROOT/loader_path).read_text())
    assert loader['current'] is False and loader['offline_only'] is True
    assert loader['payload_decoded'] is False and loader['motor_configuration_present'] is None
    assert loader['motor_firmware_version'] is None
    assert all(loader[k]==0 for k in ('motor_configuration_values_obtained','command_writes','rpc_calls','configuration_calls','calibration_calls','vendor_program_executions'))
    assert {f['sha256'] for f in loader['module_identity']['files']}=={f['sha256'] for f in fmx['files']}
    library_path='g1_console/assets/service_audit_sources/large_library_review_20261021.json'
    library_receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/large_library_receipt_20261021.json').read_text())
    for source in library_receipt['sources']+library_receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    library=json.loads((ROOT/library_path).read_text())
    assert library['current'] is False and library['offline_only'] is True
    assert library['payload_decoded'] is False and library['motor_configuration_present'] is None
    assert all(library[k]==0 for k in ('motor_configuration_values_obtained','command_writes','rpc_calls','configuration_calls','calibration_calls','vendor_program_executions','runtime_memory_reads'))
    assert library['master_linkage']['sha256']==next(f['sha256'] for f in fmx['files'] if f['name']=='master_service')
    assert len(library['files'])==2 and all(f['unchanged_during_read'] and not f['implementation_match_confirmed'] for f in library['files'])
    package_path='g1_console/assets/service_audit_sources/package_cache_review_20261021.json'
    package_receipt=json.loads((ROOT/'g1_console/assets/service_audit_sources/package_cache_receipt_20261021.json').read_text())
    for source in package_receipt['sources']+package_receipt['scripts']:
        assert digest(source['path'])==source['sha256'], source['path']
    package=json.loads((ROOT/package_path).read_text())
    assert package['motor_configuration_values_obtained']==package['command_writes']==package['rpc_calls']==0
    assert package['payload_decoded'] is False and package['motor_configuration_present'] is None
    assert package['master_identity'][0]['sha256']==library['master_linkage']['sha256']
    return dict(source=dict(path=path,sha256=digest(path)),receipt=receipt,**raw,
        configuration_analysis=dict(source=dict(path=analysis_path,sha256=digest(analysis_path)),receipt=analysis_receipt,**analysis),
        fmx_analysis=dict(source=dict(path=fmx_path,sha256=digest(fmx_path)),receipt=fmx_receipt,**fmx),
        fmx_loader_review=dict(source=dict(path=loader_path,sha256=digest(loader_path)),receipt=loader_receipt,**loader),
        large_library_review=dict(source=dict(path=library_path,sha256=digest(library_path)),receipt=library_receipt,**library),
        package_cache_review=dict(source=dict(path=package_path,sha256=digest(package_path)),receipt=package_receipt,**package),
        conclusion='MegaFlow 配置离线解析为组件和平台清单；FMX 内部字段未知；没有取得本机电机增益、限值或零位标定',
        findings=[dict(item='master_service',result='SysV 生成单元启动官方二进制，无额外 argv；历史日志记录 master_service.json 解析成功',detail='468 字节非明文文件，尚不能确认其中有电机配置；历史日志日期不是实时状态'),
            dict(item='MegaFlowRuntimeDaemon',result='实际 argv 的 -c 指向本机 bin/config.bin',detail='首次检查仅记录 14640 字节指纹；后续离线解析见本节最新结果。-d 地址不作为猜测读取协议的依据'),
            dict(item='video_hub_pc4',result='启动定义指定 cyclonedds.xml，实际视频进程存在',detail='DDS 启动配置不等于相机/电机标定，也不证明此刻图像可用'),
            dict(item='能力目录',result='MegaFlow 日志出现 motionInfo 查询文字',detail='只是目录线索，不能从 SQL 文本推定本机动作参数、可调用接口或安全性')])
