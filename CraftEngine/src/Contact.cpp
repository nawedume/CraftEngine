#include "Contact.h"
#include "ContactCapsuleVsCapsule.h"
#include "ContactCapsuleVsHull.h"
#include "ContactHullVsHull.h"
#include "ContactSphereVsCapsule.h"
#include "ContactSphereVsHull.h"
#include "ContactSphereVsSphere.h"
#include "World.h"
#include <vector>

namespace ce {

#define COLLIDER(a, b) (a << 16) | b

void DetectContacts(World *world, std::vector<BodyPair> bodyIds) {
    WorldContactSet *contactSet = world->ContactSet;
    for (BodyPair &pair : bodyIds) {

        Collider *collider1 = &world->Colliders[pair.Body1];
        Collider *collider2 = &world->Colliders[pair.Body2];

        u32 c1 = collider1->Id;
        u32 c2 = collider2->Id;

        u32 bid1 = pair.Body1;
        u32 bid2 = pair.Body2;

        if (c1 > c2) {
            std::swap(bid1, bid2);
            std::swap(c1, c2);
            std::swap(collider1, collider2);
        }
        Transform *transform1 = &world->Transforms[bid1];
        Transform *transform2 = &world->Transforms[bid2];

        u32 cid = COLLIDER(c1, c2);
        switch (cid) {
        case COLLIDER(SPHERE, SPHERE):
            SphereAndSphereTest(bid1, bid2, transform1, transform2, &collider1->Sphere, &collider2->Sphere, contactSet);
            break;
        case COLLIDER(SPHERE, CAPSULE):
            SphereAndCapsuleTest(bid1, bid2, transform1, transform2, &collider1->Sphere, &collider2->Capsule,
                                 contactSet);
            break;
        case COLLIDER(SPHERE, HULL):
            SphereAndHullTest(bid1, bid2, transform1, transform2, &collider1->Sphere, &collider2->Hull, contactSet);
            break;
        case COLLIDER(CAPSULE, CAPSULE):
            CapsuleAndCapsuleTest(bid1, bid2, transform1, transform2, &collider1->Capsule, &collider2->Capsule,
                                  contactSet);
            break;
        case COLLIDER(CAPSULE, HULL):
            CapsuleAndHullTest(bid1, bid2, transform1, transform2, &collider1->Capsule, &collider2->Hull, contactSet);
            break;
        case COLLIDER(HULL, HULL):
            HullAndHullTest(bid1, bid2, transform1, transform2, &collider1->Hull, &collider2->Hull, contactSet);
            break;
        }
    }
}
}; // namespace ce
