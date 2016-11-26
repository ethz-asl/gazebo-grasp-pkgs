#include <gazebo_state_plugins/GazeboObjectInfo.h>
#include <object_msgs_tools/ObjectFunctions.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <algorithm>
#include <assert.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#define DEFAULT_PUBLISH_OBJECTS false
#define DEFAULT_WORLD_OBJECTS_TOPIC "world/objects"
#define DEFAULT_REQUEST_OBJECTS_TOPIC "world/request_object"
#define DEFAULT_ROOT_FRAME_ID "world"

//publishing rate
#define UPDATE_RATE 5

#define OBJECT_QUEUE_SIZE 100

using gazebo::GazeboObjectInfo;

GZ_REGISTER_WORLD_PLUGIN(GazeboObjectInfo)

GazeboObjectInfo::GazeboObjectInfo() :
    WorldPlugin(){
    reGenerateObjects=true;
}

void GazeboObjectInfo::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
    ros::NodeHandle node("/gazebo_state_plugins");

    node.param<bool>("publish_world_objects", PUBLISH_OBJECTS, DEFAULT_PUBLISH_OBJECTS);
    ROS_INFO("GazeboObjInfo: Got objects publish flag: <%i>", PUBLISH_OBJECTS);

    node.param<std::string>("world_objects_topic", WORLD_OBJECTS_TOPIC, DEFAULT_WORLD_OBJECTS_TOPIC);
    ROS_INFO("GazeboObjInfo: Got objects topic name: <%s>", WORLD_OBJECTS_TOPIC.c_str());

    node.param<std::string>("request_object_service", REQUEST_OBJECTS_TOPIC, DEFAULT_REQUEST_OBJECTS_TOPIC);
    ROS_INFO("GazeboObjInfo: Got objects topic name: <%s>", REQUEST_OBJECTS_TOPIC.c_str());

    node.param<std::string>("objects_frame_id", ROOT_FRAME_ID, DEFAULT_ROOT_FRAME_ID);
    ROS_INFO("GazeboObjInfo: Got objects frame id: <%s>", ROOT_FRAME_ID.c_str());

    world=_world;

    if (PUBLISH_OBJECTS){
        update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboObjectInfo::onWorldUpdate, this));
        object_pub = node.advertise<GazeboObjectInfo::ObjectMsg>(WORLD_OBJECTS_TOPIC, OBJECT_QUEUE_SIZE);
    }

    ros::Rate rate(UPDATE_RATE);
    publishTimer=node.createTimer(rate,&GazeboObjectInfo::advertEvent, this);

    request_object_srv = node.advertiseService(REQUEST_OBJECTS_TOPIC, &GazeboObjectInfo::requestObject,this);
    request_object_static = node.advertiseService("world/object_static", &GazeboObjectInfo::requestStatic,this);

}

bool GazeboObjectInfo::requestStatic(motion_execution_msgs::SetObjectStatic::Request  &req, motion_execution_msgs::SetObjectStatic::Response &res)
{
  std::string modelName=req.name;
  physics::ModelPtr model=world->GetModel(modelName);
  model-> SetStatic(req.set_static);

  if(req.set_static) ROS_INFO("model %s was set with static", modelName.c_str());
  else ROS_INFO("model %s was set with active", modelName.c_str());
  return true;
}

bool GazeboObjectInfo::requestObject(object_msgs::ObjectInfo::Request &req, object_msgs::ObjectInfo::Response &res) {

    std::string modelName=req.name;
    physics::ModelPtr model=world->GetModel(modelName);

    if (!model.get()) {
        // ROS_ERROR("Model %s not found",modelName.c_str());
        res.success=false;
        res.error_code=object_msgs::ObjectInfo::Response::OBJECT_NOT_FOUND;
        return true;
    }

    res.error_code=object_msgs::ObjectInfo::Response::NO_ERROR;
    res.success=true;
    res.object=createBoundingBoxObject(model,req.get_geometry);
    //ROS_INFO("Received service request for object info!");
    return true;
}

void GazeboObjectInfo::advertEvent(const ros::TimerEvent& e) {
    if (object_pub.getNumSubscribers()==0) return;
    std::vector<GazeboObjectInfo::ObjectMsg>::iterator it;
    for (it=lastGeneratedObjects.begin(); it!=lastGeneratedObjects.end(); ++it) {
        object_pub.publish(*it);
    }
    lastGeneratedObjects.clear();
    reGenerateObjects=true;
}


void GazeboObjectInfo::onWorldUpdate() {
    if (object_pub.getNumSubscribers()==0) return;

    if (!reGenerateObjects) return;

    physics::Model_V models=world->GetModels();
    physics::Model_V::iterator m_it;

    bool send_shape=false;
    for (m_it=models.begin(); m_it!=models.end(); ++m_it) {
        //ROS_INFO_STREAM("Have model "<<(*m_it)->GetName());
        GazeboObjectInfo::ObjectMsg obj=createBoundingBoxObject(*m_it,send_shape);
        lastGeneratedObjects.push_back(obj);
    }

    reGenerateObjects=false;
}

shape_msgs::Mesh * GazeboObjectInfo::getMesh(physics::CollisionPtr& c){
    shape_msgs::Mesh tri;
    msgs::Geometry geom;
    physics::ShapePtr shape=c->GetShape();
    shape->FillMsg(geom);

    if(geom.has_mesh()){
    	 //const gazebo::physics::MeshShape * mesh_shape = dynamic_cast<const physics::MeshShape*>(shape.get());
       // const gazebo::physics::BoxShape * box=dynamic_cast<const physics::BoxShape*>(shape.get());

    	boost::shared_ptr<gazebo::physics::MeshShape> mesh_shape = boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(shape);
    	 std::string name_ = mesh_shape->GetName();
    	 std::string uri = mesh_shape->GetMeshURI();
    	 const gazebo::common::Mesh *mesh = gazebo::common::MeshManager::Instance()->GetMesh(uri);
         unsigned n_submeshes = mesh->GetSubMeshCount();

         for(unsigned m=0; m < n_submeshes; m++) {

           const gazebo::common::SubMesh *submesh = mesh->GetSubMesh(m);
           unsigned n_vertices = submesh->GetVertexCount();

           switch(submesh->GetPrimitiveType())
           {
             case gazebo::common::SubMesh::POINTS:
             case gazebo::common::SubMesh::LINES:
             case gazebo::common::SubMesh::LINESTRIPS:
               // These aren't supported
               break;
             case gazebo::common::SubMesh::TRIANGLES:
             	 {
                     shape_msgs::Mesh mesh_msg;
                     mesh_msg.vertices.resize(n_vertices);
                     mesh_msg.triangles.resize(n_vertices/3);

                     for(size_t v=0; v < n_vertices; v++) {
                         const int index = submesh->GetIndex(v);
                         const gazebo::math::Vector3 vertex = submesh->GetVertex(v);

                         mesh_msg.vertices[index].x = vertex.x;
                         mesh_msg.vertices[index].y = vertex.y;
                         mesh_msg.vertices[index].z = vertex.z;

                         mesh_msg.triangles[v/3].vertex_indices[v%3] = index;
                     }
                     tri = mesh_msg;
                     break;

             	 }
             case gazebo::common::SubMesh::TRISTRIPS:
             {
                 shape_msgs::Mesh mesh_msg;
                 mesh_msg.vertices.resize(n_vertices);
                 mesh_msg.triangles.resize(n_vertices-2);

                 for(size_t v=0; v < n_vertices; v++) {
                   const int index = submesh->GetIndex(v);
                   const gazebo::math::Vector3 vertex = submesh->GetVertex(v);

                   mesh_msg.vertices[index].x = vertex.x;
                   mesh_msg.vertices[index].y = vertex.y;
                   mesh_msg.vertices[index].z = vertex.z;

                   if(v < n_vertices-2) mesh_msg.triangles[v].vertex_indices[0] = index;
                   if(v > 0 && v < n_vertices-1) mesh_msg.triangles[v-1].vertex_indices[1] = index;
                   if(v > 1) mesh_msg.triangles[v-2].vertex_indices[2] = index;
                 }
                 tri = mesh_msg;
               break;
             }
             case gazebo::common::SubMesh::TRIFANS:
               // Unsupported
               gzerr << "TRIFANS not supported yet." << std::endl;
               break;
           };
         }

    }
    return new shape_msgs::Mesh(tri);
}

shape_msgs::SolidPrimitive * GazeboObjectInfo::getSolidPrimitive(physics::CollisionPtr& c) {
    shape_msgs::SolidPrimitive solid;
    msgs::Geometry geom;
    physics::ShapePtr shape=c->GetShape();
    shape->FillMsg(geom);
    if (geom.has_box()) {
        //ROS_INFO("shape type %i of collision %s is a box! ", c->GetShapeType(), c->GetName().c_str());

        const gazebo::physics::BoxShape * box=dynamic_cast<const physics::BoxShape*>(shape.get());
        if (!box) {
            ROS_ERROR("Dynamic cast failed for box shape");
            return NULL;
        }
        math::Vector3 bb=box->GetSize();
        if ((bb.x < 1e-05) || (bb.y < 1e-05) || (bb.z < 1e-05)){
            ROS_WARN_ONCE("Skipping coll %s because its bounding box is flat",c->GetName().c_str());
            return NULL;
        }
        solid.type=shape_msgs::SolidPrimitive::BOX;
        solid.dimensions.resize(3);
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_X]=bb.x;
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=bb.y;
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=bb.z;
    }/*else if (geom.has_cylinder()) {
        ROS_INFO("shape type %i of collision %s is a cylinder! ", c->GetShapeType(), c->GetName().c_str());
        const gazebo::physics::CylinderShape * cyl=dynamic_cast<const physics::CylinderShape*>(shape.get());
        if (!cyl) {
            ROS_ERROR("Dynamic cast failed for cylinder shape");
            return NULL;
        }
        solid.type=shape_msgs::SolidPrimitive::CYLINDER;
        solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=cyl->GetLength();
        solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=cyl->GetRadius();
    }*/else if (geom.has_sphere()) {
        //ROS_INFO("shape type %i of collision %s is a sphere! ", c->GetShapeType(), c->GetName().c_str());

        const gazebo::physics::SphereShape * sp=dynamic_cast<const physics::SphereShape*>(shape.get());
        if (!sp) {
            ROS_ERROR("Dynamic cast failed for cylinder shape");
            return NULL;
        }

        solid.type=shape_msgs::SolidPrimitive::SPHERE;
        solid.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=sp->GetRadius();
    }else{
        ROS_WARN("shape type %i of collision %s not supported. Using bounding box instead. ", c->GetShapeType(),c->GetName().c_str());
        math::Box box=c->GetBoundingBox();
        math::Vector3 bb (box.GetXLength(),box.GetXLength(),box.GetXLength());
        if ((bb.x < 1e-05) || (bb.y < 1e-05) || (bb.z < 1e-05)){
            ROS_WARN_ONCE("Skipping coll %s because its bounding box is flat",c->GetName().c_str());
            return NULL;
        }
        solid.type=shape_msgs::SolidPrimitive::BOX;
        solid.dimensions.resize(3);
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_X]=bb.x*0.8;
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=bb.y*0.8;
        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=bb.z*0.8;
    }
    //ROS_INFO_STREAM("Solid computed."<<std::endl<<solid);
    return new shape_msgs::SolidPrimitive(solid);
}


GazeboObjectInfo::ObjectMsg GazeboObjectInfo::createBoundingBoxObject(physics::ModelPtr& model, bool include_shape)
{
    GazeboObjectInfo::ObjectMsg obj;

    physics::Link_V links=model->GetLinks();
    physics::Link_V::iterator l_it;

    obj.name=model->GetName();
    obj.header.stamp=ros::Time::now();
    obj.header.frame_id=ROOT_FRAME_ID;
    // obj.type =  no object type given
    // the custum origin (Object::origin) is going to be set to the first link encountered
    bool origin_init = false;
    for (l_it=links.begin(); l_it!=links.end(); ++l_it)
    {
        physics::LinkPtr link=*l_it;

        std::string linkName=link->GetName();

        math::Pose link_pose=link->GetWorldPose();
        //ROS_INFO("Link for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),link_pose.pos.x,link_pose.pos.y,link_pose.pos.z);
        //ROS_INFO("Link found for model %s: %s",model->GetName().c_str(),link->GetName().c_str());

        physics::Collision_V colls=link->GetCollisions();
        physics::Collision_V::iterator cit;
        for (cit=colls.begin(); cit!=colls.end(); ++cit)
        {
            physics::CollisionPtr c=*cit;

            math::Pose rel_pose=c->GetRelativePose();

            //ROS_INFO("Collision for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),rel_pose.pos.x,rel_pose.pos.y,rel_pose.pos.z);
            //math::Pose w_pose=c->GetWorldPose();
            //ROS_INFO("World pos for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),w_pose.pos.x,w_pose.pos.y,w_pose.pos.z);

            math::Pose coll_pose=rel_pose+link_pose; //XXX c->GetWorldPose() does not work for non-static objects, so it has to be relative pose and link world pose
            geometry_msgs::Pose pose;
            pose.position.x=coll_pose.pos.x;
            pose.position.y=coll_pose.pos.y;
            pose.position.z=coll_pose.pos.z;
            pose.orientation.x=coll_pose.rot.x;
            pose.orientation.y=coll_pose.rot.y;
            pose.orientation.z=coll_pose.rot.z;
            pose.orientation.w=coll_pose.rot.w;

            if(c->GetShape()->HasType(gazebo::physics::Base::MESH_SHAPE)) obj.mesh_poses.push_back(pose);
            if(c->GetShape()->HasType(gazebo::physics::Base::BOX_SHAPE)) obj.primitive_poses.push_back(pose);

            if (!origin_init)
            {
                obj.origin = pose;
            }

            if (include_shape) {
            	if(c->GetShape()->HasType(gazebo::physics::Base::MESH_SHAPE)){
            		ROS_INFO_ONCE("Mesh was recognized");
            		shape_msgs::Mesh * mesh = getMesh(c);
            		obj.meshes.push_back(*mesh);
            		delete mesh;
            	}

            	if(c->GetShape()->HasType(gazebo::physics::Base::BOX_SHAPE)){
            		shape_msgs::SolidPrimitive * solid = getSolidPrimitive(c);
            		obj.primitives.push_back(*solid);
                    if (!solid) {
                        ROS_WARN("Skipping coll %s of link %s of model %s, could not get SolidPrimitive. ",c->GetName().c_str(),linkName.c_str(),obj.name.c_str());
                        continue;
                    }
                    delete solid;
            	}
                obj.content=GazeboObjectInfo::ObjectMsg::SHAPE;
            }else {
                obj.content=GazeboObjectInfo::ObjectMsg::POSE;
                //ROS_INFO("Pub pose for %s (%s %s)",c->GetName().c_str(),link->GetName().c_str(),model->GetName().c_str());
            }
        }
    }

    obj.primitive_origin = GazeboObjectInfo::ObjectMsg::ORIGIN_AVERAGE;
    obj.mesh_origin = GazeboObjectInfo::ObjectMsg::ORIGIN_UNDEFINED;

    return obj;
}
