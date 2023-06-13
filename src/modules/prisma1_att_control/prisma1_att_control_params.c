/**
 * Tilting controller Ki for attitude for x-y axes
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_ATTKI_XY, 0.01f);
/**
 * Tilting controller Ki for attitude for z axis
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_ATTKI_Z, 0.01f);
/**
 * Geometric tracking / tilting controller Kr for x-y axes
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KR_XY, 0.65f);
/**
 * Geometric tracking / tilting controller Kr for z axis
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KR_Z, 0.65f);
/**
 * Geometric tracking controller Kom for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KOM_XY, 0.3f);
/**
 * Geometric tracking controller Kom for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KOM_Z, 0.11f);
/**
 * Tilting controller Kq for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KQ_XY, 0.3f);
/**
 * Tilting controller Kq for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KQ_Z, 0.11f);
/**
 * Geometric tracking controller Ki_att for xy
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KI_ATT_XY, 0.06f);
/**
 * Geometric tracking controller Ki_att for z
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_KI_ATT_Z, 0.06f);
/**
 * Geometric tracking controller c2
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_C2, 0.8f);
/**
 * X component of the inertia matrix
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_IBX, 0.029125f);
/**
 * Y component of the inertia matrix
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_IBY, 0.029125f);
/**
 * Z component of the inertia matrix
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_IBZ, 0.055225f);
/**
 * Maximum thrust of the UAV
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_THR, 28.2656f);
/**
 * Maximum X torque of the UAV
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_X_TOR, 2.968f);
/**
 * Maximum Y torque of the UAV
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_Y_TOR, 1.837f);
/**
 * Maximum Z torque of the UAV
 *
 * @decimal 2
 * @group PRISMA
 */
PARAM_DEFINE_FLOAT(PRISMA_Z_TOR, 0.848f);
/**
 * Define what angle the joystick controls
 *
 * @min 0
 * @max 2
 * @group PRISMA
 * @value 0 Roll
 * @value 1 Pitch
 * @value 2 Yaw
 */
PARAM_DEFINE_INT32(PRISMA_ANG_MODE, 2);
