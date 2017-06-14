//
// Created by nima on 08/06/17.
//

#include "ODETask.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>

ODETask::ODETask(const std::string stl_file_dir,
                 const bool show_ref_frames, const bool biman,
                 const bool with_guidance)
        :
        VTKTask(show_ref_frames, biman, with_guidance),
        stl_files_dir(stl_file_dir),
        d_board_actor(vtkSmartPointer<vtkActor>::New())

{



    // -------------------------------------------------------------------------
    //  ACTIVE CONSTRAINT
    // -------------------------------------------------------------------------
    // these parameters could be set as ros parameters too but since
    // they change during the task I am hard coding them here.
    ac_parameters.method = 0; // 0 for visco/elastic
    ac_parameters.active = 0;

    ac_parameters.max_force = 4.0;
    ac_parameters.linear_elastic_coeff = 1000.0;
    ac_parameters.linear_damping_coeff = 10.0;

    ac_parameters.max_torque = 0.03;
    ac_parameters.angular_elastic_coeff = 0.04;
    ac_parameters.angular_damping_coeff = 0.002;





    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);


//    // -------------------------------------------------------------------------
//    // Create a cube for the floor
//    vtkSmartPointer<vtkCubeSource> floor_source =
//            vtkSmartPointer<vtkCubeSource>::New();
//    double floor_dimensions[3] = {0.1, 0.09, 0.001};
//    floor_source->SetXLength(floor_dimensions[0]);
//    floor_source->SetYLength(floor_dimensions[1]);
//    floor_source->SetZLength(floor_dimensions[2]);
//    vtkSmartPointer<vtkPolyDataMapper> floor_mapper =
//            vtkSmartPointer<vtkPolyDataMapper>::New();
//    floor_mapper->SetInputConnection(floor_source->GetOutputPort());
//    vtkSmartPointer<vtkActor> floor_actor = vtkSmartPointer<vtkActor>::New();
//    floor_actor->SetMapper(floor_mapper);
//    floor_actor->SetPosition(floor_dimensions[0] / 2, floor_dimensions[1] / 2,
//                             -floor_dimensions[2]);
//    floor_actor->GetProperty()->SetOpacity(0.3);
//    double DeepPink[3] {1.0, 0.08, 0.58};
//    floor_actor->GetProperty()->SetColor(DeepPink);

    // -------------------------------------------------------------------------
    // Create a cube for the board
    vtkSmartPointer<vtkCubeSource> board_source =
            vtkSmartPointer<vtkCubeSource>::New();
    board_dimensions[0]  = 0.18;
    board_dimensions[1]  = 0.14;
    board_dimensions[2]  = 0.5;

    board_source->SetXLength(board_dimensions[0]);
    board_source->SetYLength(board_dimensions[1]);
    board_source->SetZLength(board_dimensions[2]);
    vtkSmartPointer<vtkPolyDataMapper> board_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    board_mapper->SetInputConnection(board_source->GetOutputPort());
    d_board_actor->SetMapper(board_mapper);
    d_board_actor->SetPosition(board_dimensions[0] / 2,
                               board_dimensions[1] / 2,
                               -board_dimensions[2]);
    d_board_actor->GetProperty()->SetOpacity(1.00);
    double colr[3] {1.0, 1.0, 1.0};
    d_board_actor->GetProperty()->SetColor(colr);




    // -------------------------------------------------------------------------
    // Error history spheres

    vtkSmartPointer<vtkSphereSource>  source =
            vtkSmartPointer<vtkSphereSource>::New();

    source->SetRadius(RAD_SPHERES);
    source->SetPhiResolution(30);
    source->SetThetaResolution(30);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(source->GetOutputPort());

    vtkSmartPointer<vtkMinimalStandardRandomSequence> sequence =
            vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
    // initialize the sequence
    sequence->SetSeed(1);

    for (int i = 0; i < NUM_SPHERES; ++i) {

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        sequence->GetRangeValue(0.0,1.0);
        double a = sequence->GetRangeValue(0.0,1.0);
        actor->GetProperty()->SetColor(0.6 - 0.2*a, 0.6 - 0.3*a, 0.7 + 0.3*a);
        sequence->Next();

        a = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double b = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double c = sequence->GetRangeValue(0.0, 0.05);
        sequence->Next();
        sphere_positions.push_back({a,b, 0.07 + c});
        actor->SetPosition(sphere_positions[i][0],
                           sphere_positions[i][1],
                           sphere_positions[i][2]);
        actor->GetProperty()->SetSpecular(0.8);
        actor->GetProperty()->SetSpecularPower(50);
        d_sphere_actors.push_back(actor);
    }


    // -------------------------------------------------------------------------
    double source_scales = 0.006;


    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);
    }
    for (int j = 0; j < d_sphere_actors.size(); ++j) {
        actors.push_back(d_sphere_actors[j]);
    }

    actors.push_back(d_board_actor);



    dInitODE ();

    InitODE();

}


//------------------------------------------------------------------------------
void ODETask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                        const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

//------------------------------------------------------------------------------
void ODETask::UpdateActors() {

    for (int i = 0; i < 5; ++i) {
        SimLoopODE();
    }


}




//------------------------------------------------------------------------------
bool ODETask::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters ODETask::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState ODETask::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void ODETask::ResetTask() {
    ROS_INFO("Resetting the task.");
//    number_of_repetition = 0;
//    task_state = ODETaskState::RepetitionComplete;
//    ResetOnGoingEvaluation();
//    ResetScoreHistory();
}

void ODETask::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
//    if(task_state== ODETaskState::ToEndPoint ||
//       task_state == ODETaskState::ToStartPoint){
//
//        ResetOnGoingEvaluation();
//        if(number_of_repetition>0)
//            number_of_repetition--;
//        task_state = ODETaskState::RepetitionComplete;
//
//    }
}


void ODETask::FindAndPublishDesiredToolPose() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            ("/PSM1/tool_pose_desired", 10);
    if(bimanual)
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

//        CalculatedDesiredToolPose();

        // publish desired poses
        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/task_space";
            pose_msg.header.stamp = ros::Time::now();
            // publish
            pub_desired[n_arm].publish(pose_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}





void ODETask::InitODE() {


    // Create a new, empty world and assign its ID number to World. Most applications will only need one world.

//    World = dWorldCreate();

    // Create a new collision space and assign its ID number to Space, passing 0 instead of an existing dSpaceID.
    // There are three different types of collision spaces we could create here depending on the number of objects
    // in the world but dSimpleSpaceCreate is fine for a small number of objects. If there were more objects we
    // would be using dHashSpaceCreate or dQuadTreeSpaceCreate (look these up in the ODE docs)

    Space = dSimpleSpaceCreate(0);

    // Create a joint group object and assign its ID number to contactgroup. dJointGroupCreate used to have a
    // max_size parameter but it is no longer used so we just pass 0 as its argument.

//    contactgroup = dJointGroupCreate(0);

    // Create a ground plane in our collision space by passing Space as the first argument to dCreatePlane.
    // The next four parameters are the planes normal (a, b, c) and distance (d) according to the plane
    // equation a*x+b*y+c*z=d and must have length 1

    dCreatePlane(Space, 0, 0, 1, (float)-board_dimensions[2]);

    // Now we set the gravity vector for our world by passing World as the first argument to dWorldSetGravity.
    // Earth's gravity vector would be (0, -9.81, 0) assuming that +Y is up. I found that a lighter gravity looked
    // more realistic in this case.

    dWorldSetGravity(World, 0.0, 0,  (float)-9.8);

    // These next two functions control how much error correcting and constraint force mixing occurs in the world.
    // Don't worry about these for now as they are set to the default values and we could happily delete them from
    // this example. Different values, however, can drastically change the behaviour of the objects colliding, so
    // I suggest you look up the full info on them in the ODE docs.

//    dWorldSetERP(World, 0.2);
//
    dWorldSetCFM(World, 1e-5);

    // This function sets the velocity that interpenetrating objects will separate at. The default value is infinity.
    dWorldSetContactMaxCorrectingVel(World, 0.9);

    // This function sets the depth of the surface layer around the world objects. Contacts are allowed to sink into
    // each other up to this depth. Setting it to a small value reduces the amount of jittering between contacting
    // objects, the default value is 0.

    dWorldSetContactSurfaceLayer(World, 0.001);

    // To save some CPU time we set the auto disable flag to 1. This means that objects that have come to rest (based
    // on their current linear and angular velocity) will no longer participate in the simulation, unless acted upon
    // by a moving object. If you do not want to use this feature then set the flag to 0. You can also manually enable
    // or disable objects using dBodyEnable and dBodyDisable, see the docs for more info on this.

    dWorldSetAutoDisableFlag(World, 1);

    // This brings us to the end of the world settings, now we have to initialize the objects themselves.
    // Create a new body for our object in the world and get its ID.

//    Objct.Body = dBodyCreate(World);
//
//
//
//    // Next we set the position of the new body
//
//    dBodySetPosition(Objct.Body, 0.001, 0.02, 0.05);
//
//    // Here I have set the initial linear velocity to stationary and let gravity do the work, but you can experiment
//    // with the velocity vector to change the starting behaviour. You can also set the rotational velocity for the new
//    // body using dBodySetAngularVel which takes the same parameters.
//
//    double tempVect[3] = {0.0, 0.0, 0.0};
//
//    dBodySetLinearVel(Objct.Body, tempVect[0], tempVect[1], tempVect[2]);
//
//    // To start the object with a different rotation each time the program runs we create a new matrix called R and use
//    // the function dRFromAxisAndAngle to create a random initial rotation before passing this matrix to dBodySetRotation.
//    dMatrix3 R;
//
//    dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0,
//
//                       dRandReal() * 2.0 - 1.0,
//
//                       dRandReal() * 2.0 - 1.0,
//
//                       dRandReal() * 10.0 - 5.0);
//
//    dBodySetRotation(Objct.Body, R);
//
//    // At this point we could add our own user data using dBodySetData but in this example it isn't used.
//    size_t i = 0;
//
//    dBodySetData(Objct.Body, (void*)i);
//
//    // Now we need to create a box mass to go with our geom. First we create a new dMass structure (the internals
//    // of which aren't important at the moment) then create an array of 3 float (dReal) values and set them
//    // to the side lengths of our box along the x, y and z axes. We then pass the both of these to dMassSetBox with a
//    // pre-defined DENSITY value of 0.5 in this case.

    dMass m;

//    sides[0] = 0.02;
//
//    sides[1] = 0.02;
//
//    sides[2] = 0.02;
    double DENSITY = 0.005;
//    //    dMassSetBox(&m, DENSITY, sides[0], sides[1], sides[2]);
//    dMassSetSphere(&m, DENSITY, 0.01);
//    // We can then apply this mass to our objects body.
//    dBodySetMass(Objct.Body, &m);
//
//    // Here we create the actual geom object using dCreateBox. Note that this also adds the geom to our
//    // collision space and sets the size of the geom to that of our box mass.
//    //    Objct.Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
//    Objct.Geom[0] = dCreateSphere(Space, 0.01);
//
//    // And lastly we want to associate the body with the geom using dGeomSetBody. Setting a body on a geom automatically
//    // combines the position vector and rotation matrix of the body and geom so that setting the position or orientation
//    // of one will set the value for both objects. The ODE docs have a lot more to say about the geom functions.
//    dGeomSetBody(Objct.Geom[0], Objct.Body);


    for (int i = 0; i < NUM_SPHERES+1; ++i) {



        Objct[i].Body = dBodyCreate( World );

        if(i==0){
            // this is the board cube
            dBodySetPosition(Objct[i].Body,
                             (float) (board_dimensions[0] / 2.45),
                             (float) (board_dimensions[1] / 2.78),
                             (float) -board_dimensions[2]/2);

            dMassSetBox( &m,0.05,
                         (float)board_dimensions[0],
                         (float)board_dimensions[1],
                         (float)board_dimensions[2]);
            Objct[i].Geom[0] = dCreateBox( Space,
                                           (float)board_dimensions[0],
                                           (float)board_dimensions[1],
                                           (float)board_dimensions[2]);
        }
        else {
            //these are the spheres
            dMatrix3 R;

            dBodySetPosition(Objct[i].Body,
                             (float) sphere_positions[i-1][0],
                             (float) sphere_positions[i-1][1],
                             (float) sphere_positions[i-1][2]);
            dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0,
                               dRandReal() * 2.0 - 1.0,
                               dRandReal() * 2.0 - 1.0,
                               dRandReal() * 10.0 - 5.0);
            dBodySetRotation(Objct[i].Body, R);
            dBodySetData(Objct[i].Body, (void *) (size_t) i);

            dMassSetSphere(&m, DENSITY, RAD_SPHERES);
            Objct[i].Geom[0] = dCreateSphere(Space, RAD_SPHERES);
        }

        if ( Objct[i].Geom[0] )
            dGeomSetBody( Objct[i].Geom[0],Objct[i].Body );


        dBodySetMass( Objct[i].Body, &m );
    }



}

void ODETask::CloseODE() {

    // Destroy all joints in our joint group
    dJointGroupDestroy(contactgroup);

    // Destroy the collision space. When a space is destroyed, and its cleanup mode is 1 (the default)
    // then all the geoms in that space are automatically destroyed as well.
    dSpaceDestroy(Space);

    // Destroy the world and everything in it. This includes all bodies and all joints that are not part of a joint group.
    dWorldDestroy(World);


}

void ODETask::SimLoopODE() {

//    // dSpaceCollide determines which pairs of geoms in the space we pass to
//    // it may potentially intersect. We must also pass the address of a
//    // callback function that we will provide. The callback function is
//    // responsible for determining which of the potential intersections are
//    // actual collisions before adding the collision joints to our joint
//    // group called contactgroup, this gives us the chance to set the
//    // behaviour of these joints before adding them to the group. The second
//    // parameter is a pointer to any data that we may want to pass to our
//    // callback routine. We will cover the details of the nearCallback
//    // routine in the next section.
//    dSpaceCollide(Space, 0, &nearCallback);
//
//    // Now we advance the simulation by calling dWorldQuickStep. This is a faster version of dWorldStep but it is also
//    // slightly less accurate. As well as the World object ID we also pass a step size value. In each step the simulation
//    // is updated by a certain number of smaller steps or iterations. The default number of iterations is 20 but you can
//    // change this by calling dWorldSetQuickStepNumIterations.
//
//    dWorldQuickStep(World, 0.005);
//
//    // Remove all temporary collision joints now that the world has been stepped
//    dJointGroupEmpty(contactgroup);
//
//    // And we finish by calling DrawGeom which renders the objects according
////    // to their type or class
////    float pos;
////    float R;
//    DrawGeom(Objct.Geom[0], 0, 0, 0);
////    pos = *dGeomGetPosition(Object.Geom[0]);
////
////    // If there was no rotation matrix given then get the existing rotation.
////    R = *dGeomGetRotation(Object.Geom[0]);


    dSpaceCollide( Space, 0, &nearCallback );

    dWorldQuickStep( World, 0.0025 );

    for ( int j = 0; j < dSpaceGetNumGeoms( Space ); j++ )
    {
        dSpaceGetGeom( Space, j );
    }

    // remove all contact joints
    dJointGroupEmpty( contactgroup );

    for ( size_t i=0; i<NUM_SPHERES +1; i++ ) {
        if ( Objct[i].Geom[0] )
        {
            DrawGeom(Objct[i].Geom[0], 0, 0, 0, i);

        }
    }



}


void nearCallback(void *data, dGeomID o1, dGeomID o2) {

//    // Temporary index for each contact
//    int i;
//
//    // Get the dynamics body for each geom
//    dBodyID b1 = dGeomGetBody(o1);
//// Create an array of dContact objects to hold the contact joints
//    dContact contact[MAX_CONTACTS];
//
//    // Now we set the joint properties of each contact. Going into the full
//    // details here would require a tutorial of its own. I'll just say that
//    // the members of the dContact structure control the joint behaviour,
//    // such as friction, velocity and bounciness. See section 7.3.7 of the ODE
//    // manual and have fun experimenting to learn more.
//
//    for (i = 0; i < MAX_CONTACTS; i++) {
//
//        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
//
//        contact[i].surface.mu = dInfinity;
//
//        contact[i].surface.mu2 = 0;
//
//        contact[i].surface.bounce = 0.8;
//
//        contact[i].surface.bounce_vel = 0.1;
//
//        contact[i].surface.soft_cfm = 0.001;
//    }
//
//    // Here we do the actual collision test by calling dCollide. It returns
//    // the number of actual contact points or zero if there were none. As
//    // well as the geom IDs, max number of contacts we also pass the address
//    // of a dContactGeom as the fourth parameter. dContactGeom is a
//    // substructure of a dContact object so we simply pass the address of the
//    // first dContactGeom from our array of dContact objects and then pass
//    // the offset to the next dContactGeom as the fifth paramater, which is
//    // the size of a dContact structure. That made sense didn't it?
//    ODETask* self = static_cast<ODETask*>(data);
//
//    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof
//            (dContact))) {
//
//        // To add each contact point found to our joint group we call
//        // dJointCreateContact which is just one of the many
//        // different joint types available.
//        for (i = 0; i < numc; i++){
//            // dJointCreateContact needs to know which world and joint group
//            // to work with as well as the dContact object itself. It returns
//            // a new dJointID which we then use with dJointAttach to finally
//            // create the temporary contact joint between the two geom bodies.
//            dJointID c = dJointCreateContact(World, contactgroup,
//                                             contact + i);
//
//            dJointAttach(c, b1, b2);
//        }
//    }
//    dBodyID b2 = dGeomGetBody(o2);
//
//    // Create an array of dContact objects to hold the contact joints
//    dContact contact[MAX_CONTACTS];
//
//    // Now we set the joint properties of each contact. Going into the full
//    // details here would require a tutorial of its own. I'll just say that
//    // the members of the dContact structure control the joint behaviour,
//    // such as friction, velocity and bounciness. See section 7.3.7 of the ODE
//    // manual and have fun experimenting to learn more.
//
//    for (i = 0; i < MAX_CONTACTS; i++) {
//
//        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
//
//        contact[i].surface.mu = dInfinity;
//
//        contact[i].surface.mu2 = 0;
//
//        contact[i].surface.bounce = 0.8;
//
//        contact[i].surface.bounce_vel = 0.1;
//
//        contact[i].surface.soft_cfm = 0.001;
//    }
//
//    // Here we do the actual collision test by calling dCollide. It returns
//    // the number of actual contact points or zero if there were none. As
//    // well as the geom IDs, max number of contacts we also pass the address
//    // of a dContactGeom as the fourth parameter. dContactGeom is a
//    // substructure of a dContact object so we simply pass the address of the
//    // first dContactGeom from our array of dContact objects and then pass
//    // the offset to the next dContactGeom as the fifth paramater, which is
//    // the size of a dContact structure. That made sense didn't it?
//    ODETask* self = static_cast<ODETask*>(data);
//
//    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof
//            (dContact))) {
//
//        // To add each contact point found to our joint group we call
//        // dJointCreateContact which is just one of the many
//        // different joint types available.
//        for (i = 0; i < numc; i++){
//            // dJointCreateContact needs to know which world and joint group
//            // to work with as well as the dContact object itself. It returns
//            // a new dJointID which we then use with dJointAttach to finally
//            // create the temporary contact joint between the two geom bodies.
//            dJointID c = dJointCreateContact(World, contactgroup,
//                                             contact + i);
//
//            dJointAttach(c, b1, b2);
//        }
//    }
//




    int i;
    // if (o1->body && o2->body) return;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody( o1 );
    dBodyID b2 = dGeomGetBody( o2 );
    if ( b1 && b2 && dAreConnectedExcluding( b1,b2,dJointTypeContact ) ) return;

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
    for ( i=0; i<MAX_CONTACTS; i++ )
    {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.7;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.001;
    }
    if ( int numc = dCollide( o1,o2,MAX_CONTACTS,&contact[0].geom,
                              sizeof( dContact ) ) )
    {

        for ( i=0; i<numc; i++ )
        {
            dJointID c = dJointCreateContact( World,contactgroup,contact+i );
            dJointAttach( c,b1,b2 );
//            if ( show_contacts ) dsDrawBox( contact[i].geom.pos,RI,ss );
        }
    }

}



void ODETask::DrawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb,
                       size_t obj_idx) {

    // If the geom ID is missing then return immediately.
    if (!g)
        return;

    // If there was no position vector supplied then get the existing position.
    if (!pos)
        pos = dGeomGetPosition(g);

    // If there was no rotation matrix given then get the existing rotation.
    if (!R)
        R = dGeomGetRotation(g);

    if (obj_idx==0)
        d_board_actor->SetPosition(pos[0],pos[1],pos[2]);
    else
        d_sphere_actors[obj_idx-1]->SetPosition(pos[0],pos[1],pos[2]);
//    std::cout << "i: " << obj_idx <<
//              " x: " <<      pos[0] <<
//              " y: " << pos[1] <<
//              " z: " << pos[2] << std::endl;

//    int type = dGeomGetClass( g );
//    if ( type == dBoxClass )
//    {
//        dVector3 sides;
//        dGeomBoxGetLengths( g,sides );
//        dsDrawBox( pos,R,sides );
//    }
//    else if ( type == dSphereClass )
//    {
//        dsDrawSphere( pos,R,dGeomSphereGetRadius( g ) );
//    }
//

//    if ( show_aabb )
//    {
//        // draw the bounding box for this geom
//        dReal aabb[6];
//        dGeomGetAABB( g,aabb );
//        dVector3 bbpos;
//        for ( int i=0; i<3; i++ ) bbpos[i] = 0.5*( aabb[i*2] + aabb[i*2+1] );
//        dVector3 bbsides;
//        for ( int j=0; j<3; j++ ) bbsides[j] = aabb[j*2+1] - aabb[j*2];
//        dMatrix3 RI;
//        dRSetIdentity( RI );
//        dsSetColorAlpha( 1,0,0,0.5 );
//        dsDrawBox( bbpos,RI,bbsides );
//    }


//
//    d_sphere_actors[i]->SetPosition(pos[0],pos[1],pos[2]);

//    // Get the geom's class type.
//    int type = dGeomGetClass (g);
//
//    if (type == dBoxClass){
//
//        // Create a temporary array of floats to hold the box dimensions.
//        dReal sides[3];
//        dGeomBoxGetLengths(g, sides);
//
//        // Now to actually render the box we make a call to DrawBox, passing
//        // the geoms dimensions, position vector and rotation matrix. And
//        // this function is the subject of our next discussion.
//        DrawBox(sides, pos, R);
//
//    }

}

ODETask::~ODETask() {
//    CloseODE();
}


