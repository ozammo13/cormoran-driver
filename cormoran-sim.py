from cormoran import Cormoran2WD
robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 12 * 2, track_width=0.0254 * 12 * 2)
robot.main()
