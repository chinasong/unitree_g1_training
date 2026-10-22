"""Read-only standing knowledge and missing-evidence report; no command generation."""


def standing_learning(knowledge, *, current, complete, rpy, camera_live, fsm, knees):
    plan = knowledge['standing_knowledge']
    audit = plan['support_audit']
    return dict(status='unknown', already_standing=None, execution_allowed=False,
        title='尚未证实已经站稳', selected_transition=None,
        human_observation=audit['human_observation'], conditional_plan=audit['conditional_plan'],
        official_reads=audit['official_reads'], manual_review=audit['manual_review'], firmware_review=audit['firmware_review'],
        applicability_review=audit['applicability_review'],
        current_findings=[
            f'双膝角度 {knees[0]:.2f}° / {knees[1]:.2f}°；仅描述关节形态' if knees else '双膝形态缺少新鲜反馈',
            f'官方 FSM 回读 {fsm}；状态编号不证明站稳或具体动作执行' if fsm is not None else '缺少新鲜 FSM 回读'],
        evidence=[
            dict(id='body', name='身体观测', status='partial' if current and complete and rpy else 'unknown',
                 detail='关节与 IMU 可读；世界姿势、传感器外参及地面关系未核定'),
            dict(id='view', name='现场观察', status='partial' if camera_live else 'unknown',
                 detail='RGB 可读；未建立双脚、外部支撑和地面接触的可信实时验证器'),
            *[dict(id=key, name=name, status='unknown', detail=detail) for key,name,detail in plan['required_evidence']]],
        slow_stand=dict(speed_parameter=None, duration_s=None, trajectory=None,
                       firmware_support_verified=None, detail=plan['slow_stand_limitation']),
        candidates=plan['documented_transitions'], learning_steps=plan['learning_steps'],
        detail=audit['human_observation']['scope']+' 按此起点研究官方辅助预备与平衡交接，不生成轨迹或发送动作。')
