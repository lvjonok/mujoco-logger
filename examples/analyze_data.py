import matplotlib.pyplot as plt

from mujoco_logger import SimLog

log = SimLog("examples/a1_run.json")

# get metadata
print(f"Log was created at: {log.timestamp}")

# print nq, nv, nu of model
print(f"Model has {log.nq} generalized positions, {log.nv} generalized velocities, and {log.nu} control inputs")

# plot q of floating body in pinocchio convention over time
plt.plot(log.time, log.qpin[:, :7], label=["x", "y", "z", "qx", "qy", "qz", "qw"])
plt.legend()
plt.show()

# get subset of data (only second half)
N = len(log) // 2
log_subset = log[N:]

# plot q of floating body in pinocchio convention over time
plt.plot(log_subset.time, log_subset.qpin[:, :7], label=["x", "y", "z", "qx", "qy", "qz", "qw"])
plt.legend()
plt.show()
