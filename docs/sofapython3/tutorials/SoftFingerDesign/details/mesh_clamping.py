import gmsh


def define_parameters():
    mm = 1e-3
    ea1 = 8.0 * mm
    ea2 = 4.0 * mm
    La = 6.0 * mm
    la = 40.0 * mm
    inter = 15.2 * mm
    rHole = 1.7 * mm
    return ea1, ea2, La, la, inter, rHole


# Generation of the volumes for clamping the flexible finger
def generate_volume_clamping(idx, n, lc):
    # Definition of variables
    # ea1 = 8.0*mm;
    # ea2 = 4.0*mm;
    # La = 10.0*mm;
    # la = 40.0*mm;
    ea1, ea2, La, la, inter, rHole = define_parameters()

    # Definition of the points forming the contour of the clamped portion
    gmsh.model.occ.addPoint(0, -La, 0, lc, 1)
    gmsh.model.occ.addPoint(0, 0, 0, lc, 2)
    idxfin = 2 + 2 + 1 + n + 4 + n + 2
    gmsh.model.occ.addPoint(ea1, 0, 0, lc, idxfin + 1)
    gmsh.model.occ.addPoint(ea1, -La, 0, lc, idxfin + 2)

    # Definition of the points forming the contour of the fixed portion
    gmsh.model.occ.addPoint(la, 0, 0, lc, idx + 1)
    gmsh.model.occ.addPoint(la, -La, 0, lc, idx + 2)
    gmsh.model.occ.addPoint(la - ea2, -La, 0, lc, idx + 3)
    gmsh.model.occ.addPoint(la - ea2, 0, 0, lc, idx + 4)


def generate_holes_clamping(w, e1):
    # inter = 15.2*mm; # la= 40.0*mm; La = 10.0*mm; ea2 = 4.0*mm
    ea1, ea2, La, la, inter, rHole = define_parameters()

    gmsh.model.occ.addCylinder(la, -La / 2, (w - inter) / 2, -ea2, 0, 0, rHole, 10)
    gmsh.model.occ.addCylinder(la, -La / 2, (w - inter) / 2 + inter, -ea2, 0, 0, rHole, 11)

    # Substract the holes for the fixation screws
    # Definition of the cylindrical holes for the fixation
    gmsh.model.occ.cut(e1, [(3, 10), (3, 11)])

# # Draw lies between the points
# for k in range(9,12):
# 	gmsh.model.occ.addLine(k,k+1,k)

# gmsh.model.occ.addLine(12,9,12)
# gmsh.model.occ.synchronize();

# # Define the contour
# gmsh.model.occ.addCurveLoop(range(9,13),2);

# # Create the surface delimited by the contour
# surf_clamping_2 = gmsh.model.occ.addPlaneSurface([2]);

# # Extrude the surface to obtain a volume
# # the extrude() function requires a vector pair (dimension of the object , tag of the object)
# gmsh.model.occ.extrude([(2,surf_clamping_2)],0,0,w);

# gmsh.model.occ.synchronize();

