"""BO-based PD/feedforward parameter optimization pipeline.

Submodules:
  * gains_io  — best_gains.json load/save + kwargs adaptor for get_a1z_robot
                and run_test.py (--gains-file entry point).

Additional modules (search space, sampler, evaluator, watchdog) are added as
the pipeline lands — see devlog 2026-07-28 §3-5-2 for the target architecture.
"""
