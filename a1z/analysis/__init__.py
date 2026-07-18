"""a1z.analysis — joint dynamic-response + end-effector tracking test module.

See 01-docs/04-sops/03-末端轨迹跟踪与关节动态响应测试-SOP.md. Offline
sub-modules (signals/safety/metrics/report/ee_pipeline) import without any CAN
or hardware dependency; only :mod:`a1z.analysis.runner` opens the bus.
"""
