"""Vendored, editable instrument driver classes owned by the SCOPIO nodes.

Each file here is a self-contained plain-python driver: ros2_ws/ builds into a
container, so these must import nothing from outside this workspace and nothing
beyond pyvisa and the standard library. THESE are the drivers the backend runs --
edit them in place. (The repo-root DG1022Z.py is an older bench copy and is not
what the nodes load.)

A node wraps one of these classes and publishes EVERY public method of it over a
single generic `InstrumentCall` service:

    drivers/dg1022z.py  DG1022Z  <- galvo_node        srv /scopio/awg/call
    drivers/TC10LAB.py  TC10LAB  <- temperature_node  srv /scopio/temperature/call

So the ROS layer never decides which instrument features are "supported" -- the
class does, and each client app picks the subset it needs. Add a method to the
driver and it is callable the same instant. See dispatch.py.
"""
