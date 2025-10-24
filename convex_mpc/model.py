import mujoco
import mujoco.viewer
import time
import numpy as np

def inertia_ellipsoid_radii(mass, Ixx, Iyy, Izz):
    # numerically safe but should we use mujoco math functions?
    a2 = max(0.0, (5.0/(2.0*mass)) * (Iyy + Izz - Ixx))
    b2 = max(0.0, (5.0/(2.0*mass)) * (Ixx + Izz - Iyy))
    c2 = max(0.0, (5.0/(2.0*mass)) * (Ixx + Iyy - Izz))
    return np.sqrt([a2, b2, c2])

model = mujoco.MjModel.from_xml_path("unitree_robots/go2/scene.xml")
data = mujoco.MjData(model)

dt = 0.002
key_id = model.key("home").id

# Make a new camera, move it to a closer distance.
# camera = mujoco.MjvCamera()
# mujoco.mjv_defaultFreeCamera(model, camera)
# camera.distance = 2


main_name = "base_link"
bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, main_name)
mass = model.body_mass[bid]
Ixx, Iyy, Izz = model.body_inertia[bid]
axes = inertia_ellipsoid_radii(mass, Ixx, Iyy, Izz)


with mujoco.viewer.launch_passive(model, data) as viewer:
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    model.opt.timestep = dt

    ## turn off visualizing mesh geoms, can turn on by setting group to 1
    ## TODO: add some callback to toggle this during runtime and not hardcode
    viewer.opt.geomgroup[2] = 0 #  

    # Initialize the camera view to that of the free camera
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    # Toggle site frame visualization.
    # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

    while viewer.is_running():
        step_start = time.time()
        # data.ctrl = np.random.randn(model.nu) # setting random control vector for testing

        mujoco.mj_forward(model, data) # step

        ## Inertia ellipsoid visualization
        ## A bit hacky and not correct just for show for now
        ## TODO: fix properly
        
        pos = data.xipos[bid]
        R = (data.ximat[bid].reshape(3,3) if hasattr(data, "ximat") else data.xmat[bid].reshape(3,3))
        with viewer.lock():
            viewer.user_scn.ngeom = 0
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[0],
                type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                size=axes,
                pos=pos,
                mat=R.flatten(),
                rgba=np.array([1.0, 0.2, 0.2, 0.5])
            )
            viewer.user_scn.ngeom = 1

            ## Will be useful but too cluttered for now in vis
            ## TODO: figure out if there is a pre-existing toggle for this
            # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

            ## Don't use 
            ## Because it turns on inertia ellipsoid visualization for all bodies that have inertia
            # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = True
        viewer.sync()
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
