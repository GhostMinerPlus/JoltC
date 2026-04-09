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

typedef struct JPC_MassShape JPC_MassShape;

JPC_API JPC_MassShape *JPC_MassShape_new(JPC_Shape *inShape, JPC_Vec3 inCenterOfMass, JPC_MassProperties inMass);

JPC_API JPC_Shape *JPC_MassShape_GetShape(JPC_MassShape *inShape);

#ifdef __cplusplus
}
#endif
