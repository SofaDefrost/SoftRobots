# Script for the generation of a soft finger from pre-defined design parameters
# The finger consist in a planar shape that is extruded
# One end of the finger is fixed to a base, the other end to the arm of a servo motor
# Rotation of the servo motor push on the finger end, which causes the finger's deflection

# The Python API is entirely defined in the `gmsh.py' module (which contains the
# full documentation of all the functions in the API):
import gmsh
import sys
from mesh_clamping import generate_volume_clamping, generate_holes_clamping
from mesh_contact_surface import gen_contact_surface

# Before using any functions in the Python API, Gmsh must be initialized:
gmsh.initialize()

# Definition of variables
mm = 1e-3

# ## Design parameters to be varied ## #
L = 60.0 * mm
l = 40.0 * mm
e1 = 8.0 * mm
e2 = 5.0 * mm
e3 = 4.0 * mm

n = 4
d = [28. * mm, 28. * mm, 28.0 * mm, 28.0 * mm]

######################################

# Thickness of the finger / length of extrusion
w = 20.0 * mm
# Length of mesh elements
lc = 150 * mm

# Print errors if the chosen dimensions do not respect the geometry or the max printing size
Lmax = 80.0 * mm
lmax = 50.0 * mm

if L > Lmax or l > lmax:
    print("Error: Chosen dimensions are beyond the maximum fingers dimensions")
    sys.exit()
if e1 + e3 >= l:
    print("Error: The thickness' of the left and right walls are too high compared to the chosen width")
    sys.exit()
if e2 >= L:
    print("Error: The thickness at the top wall is larger than the chosen length")
    sys.exit()
if len(d) != n:
    print("Error: The numbers of intermediate points and distances are not consistent")
    sys.exit()

# Generate the inner and outer contact surface of the finger
gen_contact_surface(L, l, e1, e2, e3, n, d, w, lc)

# Create a model for the finger
gmsh.model.add("finger")

# Generate the volume required to clamp the finger on the servo-motor
idx = 1 + n + 4
generate_volume_clamping(idx, n, lc)

# Definition of the points forming the contour of the finger
# Function addPOint(x,y,z,lc,tag)
idx = 2
gmsh.model.occ.addPoint(0, L, 0, lc, idx + 1)
gmsh.model.occ.addPoint(l, L, 0, lc, idx + 2)
gmsh.model.occ.addPoint(l, L-e2, 0, lc, idx + 3)
for k in range(1, n + 1):
    gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, idx + 2 + 1 + k)

idx = 2 + 2 + 1 + n + 4
for k in range(1, n + 1):
    gmsh.model.occ.addPoint(e1 + d[n - k], k * (L - e2) / (n + 1), 0, lc, idx + k)
idx = 2 + 2 + 1 + n + 4 + n
gmsh.model.occ.addPoint(l - e3, L - e2, 0, lc, idx + 1)
gmsh.model.occ.addPoint(e1, L - e2, 0, lc, idx + 2)

gmsh.model.occ.synchronize()

nbPoint = 2 + 2 + 1 + n + 4 + n + 2 + 2

# Draw lies between the points
for k in range(1, nbPoint):
    gmsh.model.occ.addLine(k, k + 1, k)

gmsh.model.occ.addLine(nbPoint, 1, nbPoint)

gmsh.model.occ.synchronize()

# Define the contour
gmsh.model.occ.addCurveLoop(range(1, nbPoint + 1), 1)

# Create the surface delimited by the contour
surf = gmsh.model.occ.addPlaneSurface([1])

# Extrude the surface to obtain a volume
# the extrude() function requires a vector pair (dimension of the object , tag of the object)
ext1 = gmsh.model.occ.extrude([(2, surf)], 0, 0, w)

# Generate the holes for the fixation of the finger with screws
generate_holes_clamping(w, ext1)

# Select all hexahedron elements for the discretization
#
gmsh.option.setNumber("Mesh.SubdivisionAlgorithm", 0)

gmsh.model.occ.synchronize()

# Specify the mesh size
gmsh.option.setNumber("Mesh.MeshSizeMin", 0.1)
gmsh.option.setNumber("Mesh.MeshSizeMax", 0.1)

# We finally generate and save the mesh
gmsh.model.mesh.generate(1)
# gmsh.write("Data/finger.stl")

gmsh.model.mesh.generate(2)
gmsh.write("Data/finger.stl")

gmsh.model.mesh.generate(3)
gmsh.write("Data/finger.msh")

# Need to synchronize with Gmsh for it to create the desired data structures
gmsh.model.occ.synchronize()

# To visualize the model we can run the graphical user interface with
# `gmsh.fltk.run()'. Here we run it only if "-nopopup" is not provided in the
# command line arguments:
if '-nopopup' not in sys.argv:
    gmsh.fltk.run()

# This should be called when you are done using the Gmsh Python API:
gmsh.finalize()
