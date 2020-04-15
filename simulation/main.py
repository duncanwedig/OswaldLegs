import time
from simulation import geometry
from simulation import model
import pychrono as chrono
import pychrono.irrlicht as chronoirr


def main():
    mysystem = chrono.ChSystemNSC()

    model.make_model(mysystem)

    # ---------------------------------------------------------------------
    #
    #  Create an Irrlicht application to visualize the system
    #

    myapplication = chronoirr.ChIrrApp(mysystem, 'Oswald', chronoirr.dimension2du(1024, 768))

    # myapplication.AddTypicalSky()
    # myapplication.AddShadowAll()
    myapplication.AddTypicalLogo()
    myapplication.AddTypicalCamera(chronoirr.vector3df(0.6, 0.6, 0.8))
    myapplication.AddTypicalLights()

    # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    # If you need a finer control on which item really needs a visualization proxy in
    # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    myapplication.AssetBindAll();

    # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    myapplication.AssetUpdateAll();

    # ---------------------------------------------------------------------
    #
    #  Run the simulation
    #

    myapplication.SetTimestep(0.005)
    myapplication.SetTryRealtime(True)

    while myapplication.GetDevice().run():
        myapplication.BeginScene()
        myapplication.DrawAll()
        myapplication.DoStep()
        myapplication.EndScene()


if __name__ == "__main__":
    main()
