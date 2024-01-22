//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection obj_isect = intersect(ray);    // ray 与 object 的交点

    if(!obj_isect.happened) {
        /* 光线未发生碰撞*/
        return Vector3f(0, 0, 0);
    }

    if (obj_isect.m->hasEmission()) {
        // 直接打到光源上
        return obj_isect.m->getEmission();
    }

    Vector3f L_dir = Vector3f(0, 0, 0);
    Vector3f L_indir = Vector3f(0, 0, 0);


    Vector3f p = obj_isect.coords;  // ray 在 scene 上的交点
    Material* m = obj_isect.m;
    Vector3f N = obj_isect.normal.normalized();
    Vector3f wo = ray.direction;

    // 在光源上进行光线的采样
    Intersection light_isect;
    float light_pdf;
    sampleLight(light_isect, light_pdf);

   
    float eps = 1e-4;


    Vector3f light_dir = light_isect.coords - obj_isect.coords;

    float length = dotProduct(light_dir, light_dir);
    
    Vector3f light_dir_normal = light_dir.normalized();
    Vector3f f_r = obj_isect.m->eval(ray.direction, light_dir_normal, obj_isect.normal.normalized());  // BRDF
    float cos_theta = dotProduct(light_dir_normal, obj_isect.normal.normalized());
    float cos_theta_x = dotProduct(-light_dir_normal, light_isect.normal);

    Ray ray_to_light(obj_isect.coords, light_dir_normal);
    Intersection ray_light_isect = intersect(ray_to_light);

    
    if(ray_light_isect.distance - light_dir.norm() > -eps) {
        // 判断光源和交点的连线间没有遮挡
        L_dir = light_isect.emit * f_r * cos_theta * cos_theta_x / length / light_pdf;
    }



    if (get_random_float() < RussianRoulette) { // 俄罗斯轮盘赌
        
         Vector3f wi = obj_isect.m->sample(ray.direction, obj_isect.normal.normalized()).normalized();
        Ray new_input_ray = Ray(obj_isect.coords, wi);
        Intersection new_obj_isect = intersect(new_input_ray);

        if(new_obj_isect.happened && !new_obj_isect.m->hasEmission()) {
            // 新采样的光线和 object 发生碰撞
            // 要在原碰撞点的基础上进行采样而不是在新生成的下一个碰撞点上进行采样
            float obj_pdf = obj_isect.m->pdf(ray.direction, wi, obj_isect.normal.normalized());
            Vector3f f_r = obj_isect.m->eval(ray.direction, wi, obj_isect.normal.normalized());
            if(obj_pdf > eps) {
            L_indir = castRay(new_input_ray, depth+1) * f_r * dotProduct(wi, obj_isect.normal.normalized()) / obj_pdf / RussianRoulette;
            }
        }
        
    }


    return L_dir + L_indir;

}

