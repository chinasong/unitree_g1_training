"""Evidence interpretation, never a balance controller or an inferred safe set."""
import math
from .standing import standing_learning

PROJECT_TILT_RAD = .35
PROJECT_MOTOR_TEMP_C = 70


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, size):
    return isinstance(value, (tuple, list)) and len(value) == size and all(finite(v) for v in value)


def history_sample(telemetry, stamp):
    joints = telemetry.get('joints', [])
    speed = [abs(j['dq']) for j in joints if finite(j.get('dq'))]
    imu = telemetry.get('imu') or {}
    return dict(at=stamp, max_dq=max(speed) if speed else None,
                q={j['index']:j['q'] for j in joints if finite(j.get('q'))},
                mode_pr=telemetry.get('mode_pr'), mode_machine=telemetry.get('mode_machine'),
                rpy=imu.get('rpy') if vector(imu.get('rpy'), 3) else None)


def interpret(telemetry, fresh, fsm_fresh, knowledge, history=(), mode='disconnected', camera=None):
    """Unknown support/contact must survive quiet telemetry, known FSM and model matches."""
    fresh = fresh and mode == 'robot'
    fsm_fresh = fresh and fsm_fresh
    joints = {j['index']:j for j in telemetry.get('joints', [])}
    expected_indices = {m['index'] for m in knowledge['motors']}
    complete = fresh and set(joints) == expected_indices and all(
        finite(j.get('q')) and finite(j.get('dq')) for j in joints.values())
    imu = telemetry.get('imu') or {}
    rpy = imu.get('rpy') if fresh and vector(imu.get('rpy'), 3) else None
    knees = [joints.get(i, {}).get('q') for i in (3,9)]
    knees = [math.degrees(v) for v in knees] if fresh and all(finite(v) for v in knees) else None
    fsm = telemetry.get('fsm') if fsm_fresh else None
    label = knowledge['fsm_sdk_mapping'].get(str(fsm)) if fsm_fresh else None
    window = list(history) if fresh else []
    speeds = [h['max_dq'] for h in window if finite(h.get('max_dq'))]
    spans = []
    for index in expected_indices:
        values = [h.get('q', {}).get(index) for h in window]
        values = [v for v in values if finite(v)]
        if len(values) >= 2:
            spans.append(max(values)-min(values))
    temperatures = [v for j in joints.values() for v in j.get('temperature') or [] if finite(v)] if fresh else []
    thermal_complete = fresh and set(joints) == expected_indices and all(vector(j.get('temperature'), 2) for j in joints.values())
    tilt = max(abs(v) for v in rpy[:2]) if rpy else None
    feedback = telemetry.get('safety_feedback', {}) if fresh else {}
    # Transport arrival is evidence of a received message, never of calibrated sensors or health.
    feedback = {k:dict(v, current=False, raw=None) if not v.get('current') else v for k,v in feedback.items()}
    raw = lambda key: (feedback.get(key) or {}).get('raw') or {}
    secondary_rpy = raw('secondary_imu').get('rpy')
    secondary_rpy = secondary_rpy if vector(secondary_rpy, 3) else None
    camera = camera or {}
    camera_live = fresh and camera.get('status') == 'live' and finite(camera.get('age_ms')) and 0 <= camera['age_ms'] < 1000
    acquired = {
        'support': ['完整关节反馈可描述屈曲形态；关节力矩不是足底载荷'] if complete else [],
        'balance': ['已取得机体 IMU 和关节速度；未取得可信质心/支撑关系'] if rpy and complete else [],
        'terrain': [f'官方 RGB 实时画面，帧龄 {camera["age_ms"]} ms；没有深度/接触证据'] if camera_live else [],
        'calibration': ['已绑定本机型号与坐标证据；253 个实机配置字段仍为空'],
        'health': [f'{key} 已收到匹配 HG 类型的原始消息，帧龄 {feedback[key]["age_ms"]} ms；未核定单位/故障位与物理发布者' for key in ('bms','mainboard') if raw(key)],
        'recovery': [f'CheckMode 原始回读：{raw("motion_mode")}；仅为模式标识，不证明平衡控制健康'] if raw('motion_mode') else [],
        'action_contract': [f'HG 任务原始字段：{raw("sport")}；任务编号和时间不能映射为已确认的具体动作'] if raw('sport') else [],
    }
    if secondary_rpy:
        acquired['balance'].append('已收到躯干 IMU；安装外参、传感器健康和支撑关系仍未核定')
    for key in ('pelvis_odom', 'torso_odom'):
        if raw(key):
            acquired['balance'].append(f'{key} 已收到位姿/速度估计；odom 原点与地面的关系、协方差可信度未核定，不能当作离地高度或平衡裕量')
    if raw('fsm_mode'):
        acquired['action_contract'].append(f'GetFsmMode 专用只读回读：{raw("fsm_mode")}；不确认具体动作正在执行')
    checks = []

    def add(key, name, status, detail, source):
        checks.append(dict(id=key, name=name, status=status, detail=detail, source=source))

    add('real_feedback', '真实身体反馈', 'matched' if complete and mode == 'robot' else 'unknown',
        '23 个关节位置、速度为新鲜实机反馈' if complete and mode == 'robot' else '需完整、有限、新鲜的实机反馈；演示数据不算实机证据', 'rt/lowstate + CRC / tick / 时效检查')
    add('walking_fsm', '行走模式前提', 'unknown' if not fsm_fresh else 'matched' if fsm == 500 else 'violated',
        f'FSM={fsm}；500 只符合本项目行走前提，不能证明平衡或动作正在执行' if fsm_fresh else 'FSM 缺失或超过 1.5 秒', '固定版本 SDK Start→500；仅适用于行走候选任务')
    add('tilt_guard', '项目姿态拦截线', 'unknown' if tilt is None else 'violated' if tilt > PROJECT_TILT_RAD else 'matched',
        f'横滚/俯仰最大绝对值 {math.degrees(tilt):.2f}°；项目拦截线 {PROJECT_TILT_RAD} rad，未核定为防摔范围' if tilt is not None else 'IMU 缺失、过期或不是有限三轴数据', '本项目软件保护；不是厂商安全限值')
    add('thermal_guard', '项目电机温度拦截线', 'unknown' if not thermal_complete else 'violated' if max(temperatures) >= PROJECT_MOTOR_TEMP_C else 'matched',
        f'最高 {max(temperatures):g}°C；项目拦截线 {PROJECT_MOTOR_TEMP_C}°C，实机热保护与降额未知' if thermal_complete else '需要完整 23 电机双温度通道', '本项目软件保护；不是厂商安全工作温度')
    for key, name, detail in [
        ('support', '足底接触与承重', '尚无可信足底接触、载荷、支撑面；不能判断脚是否落地或被外部支撑'),
        ('balance', '动态平衡裕量', '缺少全身位姿/速度、质心与支撑关系；IMU 平稳不证明下一步稳定'),
        ('terrain', '现场地面与净空', '摩擦、坡度、台阶、地面断层和全身净空未核验；图像或二维地图单独不足'),
        ('calibration', '实机电机限值与标定', '逐电机固件限值、零位、增益和转矩转速能力未知；URDF 与示例值不能替代'),
        ('health', '驱动健康与电池余量', '仍缺本固件故障位表、降额规则及 BMS 单位/通道与物理来源核验；原始 SOC 与状态码不作为健康或续航证明'),
        ('recovery', '保护与失联处置', '现场防跌落保护、官方平衡控制归属、急停和通信失效行为尚未验证；不自动发送卸力或模式切换'),
        ('action_contract', '具体能力全过程边界', '每项能力的进入条件、执行过程、退出/打断和恢复条件均需单独核定；目录匹配不足以放行')]:
        add(key, name, 'unknown', detail, '新鲜只读消息与固定版本官方定义；剩余条件仍需本机/现场证据')
        checks[-1].update(acquired=acquired[key] if fresh else [], missing=detail)
    unknown = [c['name'] for c in checks if c['status'] == 'unknown']
    violated = [c['name'] for c in checks if c['status'] == 'violated']
    posture = ('双膝显著屈曲' if knees and all(v > 90 for v in knees) else '关节形态见角度反馈') if knees else '关节形态未知'
    mode_consistency = 'unknown'
    sport = raw('sport')
    if fsm_fresh and type(sport.get('fsm_id')) is int and type(sport.get('fsm_mode')) is int and type(raw('fsm_mode').get('data')) is int:
        mode_consistency = 'matched' if sport['fsm_id'] == fsm and sport['fsm_mode'] == raw('fsm_mode')['data'] else 'violated'
    return dict(schema_version=4, current=bool(fresh), evidence_mode=mode,
        standing=standing_learning(knowledge, current=fresh, complete=complete, rpy=rpy, camera_live=camera_live, fsm=fsm, knees=knees),
        feedback=feedback, camera_evidence=dict(current=camera_live, age_ms=camera.get('age_ms') if camera_live else None,
            detail='RGB 只确认可见环境；不自动判断足底载荷、地面摩擦和全身净空'),
        historical_visual=knowledge.get('safety_evidence_audit', {}).get('visual_observation'),
        control_mode=dict(fsm=fsm, sdk_label=label, active_capability_id=None,
            feedback_consistency=mode_consistency,
            activity_verified=False, detail='FSM 只表明服务回读的状态 ID；未取得具体能力执行中/完成的可信反馈。ZeroTorque ID 也不证明每个电机实际力矩为零'),
        posture=dict(description=posture, knee_deg=knees, imu_rpy_deg=[math.degrees(v) for v in rpy] if rpy else None,
            secondary_imu_rpy_deg=[math.degrees(v) for v in secondary_rpy] if secondary_rpy else None,
            whole_body_pose='unknown', support='unknown',
            feet=dict(left=dict(contact=None, load_n=None), right=dict(contact=None, load_n=None)),
            detail='双膝 >90° 仅为形态描述规则；未确定站/坐/卧、身体离地高度或接触对象。三维模型按关节角度显示，不表示世界中的支撑姿势'),
        observation_window=dict(samples=len(window), seconds=round(window[-1]['at']-window[0]['at'],2) if len(window)>1 else 0,
            max_abs_dq_rad_s=max(speeds) if speeds else None,
            max_joint_span_deg=math.degrees(max(spans)) if spans else None,
            certified_safe=False, learned_safe_range=None,
            detail='短时被动观察，只描述已观测变化；不把未摔倒样本、最小/最大角度或零速度学习为安全边界'),
        safety=dict(status='unverified', execution_allowed=False, fall_free_guarantee=False,
            validated_action_envelopes=knowledge['state_safety']['validated_action_envelopes'], checks=checks,
            unknown_conditions=unknown, violated_conditions=violated,
            detail='防摔动作边界尚未核定，真实运动锁定；通过个别拦截检查也不授予权限'),
        task_suggestions=[dict(title='继续只读观察和补齐证据', allowed=True, kind='read_only',
            detail='记录状态变化、官方能力条件和缺失反馈；学习任务策略，不试探电机极限或训练新运动模型'),
            dict(title='暂缓站起、行走、摆臂与姿势切换', allowed=False, kind='motion',
            detail='先核定支撑、健康、现场与该能力完整安全契约；不得由 Agent 或一项人工勾选自动放行')])
