#pragma once

#ifndef ENSURE_TESTS
#define ENSURE_EQUAL(a, b)
#define ENSURE_ENUM_EQ(a, b)
#define ENSURE_SIZE_ALIGN(a, b)
#define ENSURE_FIELD(a, b, c, d)
#define ENSURE_NORMAL_FIELD(a, b)
#endif

#include <JoltC/Enums.h>
#include <JoltC/Functions.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct JPC_ExtendShape JPC_ExtendShape;

JPC_API JPC_ExtendShape *JPC_ExtendShape_new(JPC_Shape *inShape, uint64_t inUserData, JPC_Vec3 inCenterOfMass, JPC_MassProperties inMass);

JPC_API JPC_Shape *JPC_ExtendShape_GetShape(JPC_ExtendShape *inShape);

#ifdef __cplusplus
}
#endif
