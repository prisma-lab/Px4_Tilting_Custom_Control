
PARAM_DEFINE_FLOAT(P1C_VELD_LP, 5.0f);
PARAM_DEFINE_FLOAT(VELD_LP, 5.0f);
/**
 * Geometric tracking controller Kp for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KP_XY, 8.0f);
/**
 * Geometric tracking controller Kp for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KP_Z, 20.0f);
/**
 * Geometric tracking controller Kd for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KD_XY, 1.5f);
/**
 * Geometric tracking controller Kd for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KD_Z, 10.0f);
/**
 * Geometric tracking controller Ki for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KI_XY, 1.28f);
/**
 * Geometric tracking controller Ki for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KI_Z, 1.28f);
/**
 * Geometric tracking controller c1
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_C1, 3.6f);
/**
 * Geometric tracking controller sigma
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_SIGMA, 1.0f);
/**
 * Mass of the UAV for the geometric tracking controller
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_MASS, 1.5f);
/**
 * Starting value for the Z position integral, which should help slowing down takeoff
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_Z_INT_S, 0.0f);
