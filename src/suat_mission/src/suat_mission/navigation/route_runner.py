"""Stop-on-failure sequence control, testable without ROS or hardware."""


def run_sequence(points, plan, execute, results, cancel_first=False):
    for index,item in enumerate(points):
        try:
            current_plan=plan(item)
        except Exception as exc:
            results.append({'name':item['name'],'passed':False,'motion_goal_sent':False,
                            'failure_stage':'live_replan','error':str(exc)})
            raise
        leg=execute(item,current_plan,cancel_first and index==0)
        results.append(leg)
        if not leg.get('passed'):
            raise ValueError('route stopped at '+item['name'])
        if cancel_first:
            return {'route_completed':False,'cancel_test_passed':True}
    return {'route_completed':True,'cancel_test_passed':False}
