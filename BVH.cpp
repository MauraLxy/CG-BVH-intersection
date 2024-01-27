#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives, 0);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, int dim)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]}, dim);
        node->right = recursiveBuild(std::vector{objects[1]}, dim);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {

        // TODO: modify this function to make rendering faster, e.g. using k-d tree
        // hint: make use of dim variable

        // original
        // std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //     return f1->getBounds().Centroid().x <
        //             f2->getBounds().Centroid().x;
        // });

        // optimised
        int axis = dim % 3; 
        if(axis == 0){
            // along x axis
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
        }else if(axis == 1){
            // along y axis
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
        }else{
            // along z axis
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
        }
        dim += 1;

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes, dim);
        node->right = recursiveBuild(rightshapes, dim);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{


    Intersection null_inter; // null intersection by default

    Vector3f indiv(1.0f/ray.direction[0], 1.0f/ray.direction[1], 1.0f/ray.direction[2]);

    std::array<int, 3> dirIsNeg = { {int(ray.direction.x>0), int(ray.direction.y>0), int(ray.direction.z>0)}};


    // TODO: Traverse the BVH to find intersection. you might want to call Bounds3::IntersectP, 
    // or Object::getIntersection, or recursively BVHAccel::getIntersection

    // Ray misses the node
    if (!node->bounds.IntersectP(ray, indiv, dirIsNeg))
        return null_inter;
        
    // Leaf node
    if (node->left == nullptr && node->right == nullptr) {
        return node->object->getIntersection(ray);
    }

    // Recursive traversal of the BVH tree
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    // Return the closest intersection
    if (hit1.happened && hit2.happened) {
        return (hit1.distance < hit2.distance) ? hit1 : hit2;
    }
    else if (hit1.happened) {
        return hit1;
    }
    else {
        return hit2;
    }

}