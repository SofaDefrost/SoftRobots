# Script for the generation of internal contact surfaces in the soft finger
# The surface depends on some of the finger's design parameters

import gmsh
from mesh_clamping import define_parameters
import sys


def gen_contact_surface(L, l, e1, e2, e3, n, d, w, lc):
    ea1, ea2, La, la, inter, rHole = define_parameters();

    # # Before using any functions in the Python API, Gmsh must be initialized:
    # gmsh.initialize()

    # Create a model for the finger surface contact 1
    gmsh.model.add("finger_contact_surface_in1")

    gmsh.model.occ.addPoint(ea1, 0, 0, lc, 1)
    gmsh.model.occ.addPoint(e1, L - e2, 0, lc, 2)
    gmsh.model.occ.addPoint(e1, L - e2, w, lc, 3)
    gmsh.model.occ.addPoint(ea1, 0, w, lc, 4)

    # Draw lies between the points
    for k in range(1, 4):
        gmsh.model.occ.addLine(k, k + 1, k)
    gmsh.model.occ.addLine(4, 1, 4)

    # Define the contour
    gmsh.model.occ.addCurveLoop(range(1, 5), 1);

    # Create the surface delimited by the contour
    surf = gmsh.model.occ.addPlaneSurface([1]);

    gmsh.model.occ.synchronize();

    # We finally generate and save the mesh
    gmsh.model.mesh.generate(1)
    # gmsh.write("Data/finger.stl")

    gmsh.model.mesh.generate(2)

    gmsh.option.setNumber("Mesh.MeshSizeMin", 1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 1)

    gmsh.write("Data/finger_surface_contact_in1.stl")

    # Need to synchronize with Gmsh for it to create the desired data structures
    gmsh.model.occ.synchronize()

    # # To visualize the model we can run the graphical user interface with
    # # `gmsh.fltk.run()'. Here we run it only if "-nopopup" is not provided in the
    # # command line arguments:
    # if '-nopopup' not in sys.argv:
    #     gmsh.fltk.run()

    # Create a model for the finger surface contact 2
    gmsh.model.add("finger_contact_surface_in2")

    gmsh.model.occ.addPoint(l - e3, L - e2, 0, lc, 1)
    gmsh.model.occ.addPoint(l - e3, L - e2, w, lc, 2)

    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, 2 * k + 1)
        gmsh.model.occ.addPoint(e1 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), w, lc, 2 * k + 2)

    # Draw lies between the points
    for k in range(1, n + 1):
        gmsh.model.occ.addLine(2 * (k - 1) + 1, 2 * (k - 1) + 2, 4 * (k - 1) + 1)
        gmsh.model.occ.addLine(2 * (k - 1) + 2, 2 * (k - 1) + 4, 4 * (k - 1) + 2)
        gmsh.model.occ.addLine(2 * (k - 1) + 4, 2 * (k - 1) + 3, 4 * (k - 1) + 3)
        gmsh.model.occ.addLine(2 * (k - 1) + 3, 2 * (k - 1) + 1, 4 * (k - 1) + 4)
    #     for j in range(1,5):
    #         gmsh.model.occ.addLine(k, k + 1, 4*(k-1)+1)
    # gmsh.model.occ.addLine(2 * n + 2, 1, 2 * n + 2)

    for k in range(1, n + 1):
        # Define the contour
        gmsh.model.occ.addCurveLoop(range(4 * (k - 1) + 1, 4 * (k - 1) + 5), k);

        # Create the surface delimited by the contour
        surf = gmsh.model.occ.addPlaneSurface([k]);

    gmsh.model.occ.synchronize();

    # We finally generate and save the mesh
    gmsh.model.mesh.generate(1)
    # gmsh.write("Data/finger.stl")

    gmsh.model.mesh.generate(2)
    gmsh.write("Data/finger_surface_contact_in2.stl")

    # Need to synchronize with Gmsh for it to create the desired data structures
    gmsh.model.occ.synchronize()

    # To visualize the model we can run the graphical user interface with
    # `gmsh.fltk.run()'. Here we run it only if "-nopopup" is not provided in the
    # command line arguments:
    # if '-nopopup' not in sys.argv:
    #     gmsh.fltk.run()

    # Create a model for the finger surface contact 3
    gmsh.model.add("finger_contact_surface_out")

    gmsh.model.occ.addPoint(l, L, 0, lc, 1)
    gmsh.model.occ.addPoint(l, L, w, lc, 2)
    gmsh.model.occ.addPoint(l, L-e2, 0, lc, 3)
    gmsh.model.occ.addPoint(l, L-e2, w, lc, 4)

    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, 2 * (k+1) + 1)
        gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), w, lc, 2 * (k+1) + 2)

    # Draw lies between the points
    for k in range(1, n + 2):
        gmsh.model.occ.addLine(2 * (k - 1) + 1, 2 * (k - 1) + 2, 4 * (k - 1) + 1)
        gmsh.model.occ.addLine(2 * (k - 1) + 2, 2 * (k - 1) + 4, 4 * (k - 1) + 2)
        gmsh.model.occ.addLine(2 * (k - 1) + 4, 2 * (k - 1) + 3, 4 * (k - 1) + 3)
        gmsh.model.occ.addLine(2 * (k - 1) + 3, 2 * (k - 1) + 1, 4 * (k - 1) + 4)
    #     for j in range(1,5):
    #         gmsh.model.occ.addLine(k, k + 1, 4*(k-1)+1)
    # gmsh.model.occ.addLine(2 * n + 2, 1, 2 * n + 2)

    for k in range(1, n + 2):
        # Define the contour
        gmsh.model.occ.addCurveLoop(range(4 * (k - 1) + 1, 4 * (k - 1) + 5), k);

        # Create the surface delimited by the contour
        surf = gmsh.model.occ.addPlaneSurface([k]);

    gmsh.model.occ.synchronize();

    # Specify the mesh size, higher for contact surfaces to minimize the number of elements
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.2)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 0.2)

    # We finally generate and save the mesh
    gmsh.model.mesh.generate(1)

    gmsh.model.mesh.generate(2)
    gmsh.write("Data/finger_surface_contact_out.stl")

    # Need to synchronize with Gmsh for it to create the desired data structures
    gmsh.model.occ.synchronize()

    # To visualize the model we can run the graphical user interface with
    # `gmsh.fltk.run()'. Here we run it only if "-nopopup" is not provided in the
    # command line arguments:
    # if '-nopopup' not in sys.argv:
    #     gmsh.fltk.run()

    # # This should be called when you are done using the Gmsh Python API:
    # gmsh.finalize()
