"""Shared-world task geometry that both the MuJoCo team world and the harness
tower verifier must agree on. Kept free of mujoco imports so the harness can
read it without loading the simulator."""

# Tower site sits in R3's spawn row, outside the sampled block rows.
TEAM_STACK_SITE = (0.72, 0.42)
# A staged base counts when it rests within this radius of the site.
TEAM_STACK_SITE_TOLERANCE_M = 0.025
