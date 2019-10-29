package algorithms.Astar.ODStar;

/**
 * All possibles moves in a 26 voxel neighborhood
 */
public enum Move {
    /*
     * Encoding - 3 dimensional 26 neighbor voxel with a wait move
     * A := -x
     * B := -Y
     * C := -z
     * X := +x
     * Y := +y
     * Z := +z
     * W := wait
     * N := no move yet
     */
    W, X, Y, Z, A, B, C,
    XY, XZ, YZ, XB, XC, AY, AZ, AB, AC, YC, ZB,
    XYC, XBZ, XBC, AYZ, ABZ, AYC, ABC, XYZ,
    N
}
