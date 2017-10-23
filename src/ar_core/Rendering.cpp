//
// Created by nima on 4/12/17.
//
#include "Rendering.h"
#include "VTKConversions.h"


// helper function for debugging light related issues
void AddLightActors(vtkRenderer *r);


Rendering::Rendering(ros::NodeHandle *n,
                     bool AR_mode, uint num_windows, bool with_shaodws,
                     bool offScreen_rendering,
                     std::vector<int> window_position)
        :
        num_render_windows_(num_windows),
        with_shadows_(with_shaodws),
        ar_mode_(AR_mode)
{
    // make sure the number of windows are alright
    if(num_render_windows_ <1) num_render_windows_ =1;
    else if(num_render_windows_ >2) num_render_windows_ = 2;


    std::string left_cam_name;
    if (n->getParam("left_cam_name", left_cam_name))
        cameras[0] = new CalibratedCamera(n, left_cam_name);
    else {
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as "
                        "<cam_name>_intrinsics.yaml",
                n->resolveName("left_cam_name").c_str());
    }
    std::string right_cam_name;
    if (n->getParam("right_cam_name", right_cam_name))
        cameras[1] = new CalibratedCamera(n, right_cam_name);
    else {
        ROS_ERROR(
                "Parameter '%s' is required. Place the intrinsic calibration "
                        "file of each camera in ~/.ros/camera_info/ named as "
                        "<cam_name>_intrinsics.yaml",
                n->resolveName("right_cam_name").c_str());
    }


    double view_port[3][4] = {{0.333, 0.0, 0.666, 1.0}
            , {0.666, 0.0, 1.0, 1.0}
            ,{0.0, 0.0, 0.333, 1.0}};

    render_window_[0] = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_[0]->BordersOff();

    if(num_render_windows_==1)
        render_window_[0]->SetPosition(window_position[0], window_position[1]);
    else if(num_render_windows_==2) {
        render_window_[1] = vtkSmartPointer<vtkRenderWindow>::New();
        render_window_[1]->SetPosition(window_position[2], window_position[3]);
        render_window_[1]->BordersOff();
    }

    // line 182 of vtkShadowMapPass.cxx
    lights[0] =   vtkSmartPointer<vtkLight>::New();
    lights[1] =   vtkSmartPointer<vtkLight>::New();

    double lights_focal_point[3] {0.07, -0.30, -0.3};
    double lights_position[3] {0.08, 0.7, 0.7};

    lights[0]->SetPosition(lights_position[0], lights_position[1],
                           lights_position[2]);
    lights[0]->SetFocalPoint(lights_focal_point[0], lights_focal_point[1],
                             lights_focal_point[2]);
    //lights[0]->SetColor(1.0,1.0,1.0);
    lights[0]->SetPositional(1);
    lights[0]->SetConeAngle(15);
    //lights[0]->SetLightTypeToCameraLight( );
    lights[0]->SetExponent(60);
    lights[0]->SetSwitch(1);

    lights[1]->SetPosition(lights_position[0]+0.02, lights_position[1],
                           lights_position[2]);
    lights[1]->SetFocalPoint(lights_focal_point[0], lights_focal_point[1],
                             lights_focal_point[2]);
    lights[1]->SetPositional(1);
    lights[1]->SetConeAngle(15);
    //lights[1]->SetLightTypeToCameraLight();
    lights[1]->SetExponent(60);
    lights[1]->SetSwitch(1);


    for (int i = 0; i < 2; ++i) {


        scene_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();

        // In AR the background renderer shows the real camera images from
        // the world
        background_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
        background_renderer_[i]->InteractiveOff();
        background_renderer_[i]->SetLayer(0);

        background_renderer_[i]->SetActiveCamera(cameras[i]->camera_real);

        // in AR we do not need interactive windows or dual layer render
        scene_renderer_[i]->InteractiveOff();
        scene_renderer_[i]->SetLayer(1);


        if(num_render_windows_==1){
            background_renderer_[i]->SetViewport(view_port[i]);
            scene_renderer_[i]->SetViewport(view_port[i]);
        }

        scene_renderer_[i]->AddLight(lights[0]);
        scene_renderer_[i]->AddLight(lights[1]);

        //scene_renderer_[i]->ResetCamera();
        cameras[i]->camera_virtual=
                scene_renderer_[i]->GetActiveCamera();

        if(with_shadows_)
            AddShadowPass(scene_renderer_[i]);


        int j=0;
        if(num_render_windows_==2)
            j=i;

        render_window_[j]->SetNumberOfLayers(2);
        render_window_[j]->AddRenderer(background_renderer_[i]);

        render_window_[j]->AddRenderer(scene_renderer_[i]);

        window_to_image_filter_[j] =
                vtkSmartPointer<vtkWindowToImageFilter>::New();
        window_to_image_filter_[j]->SetInput(render_window_[j]);
        //    window_to_image_filter_->SetInputBufferTypeToRGBA(); //record  he
        // alpha (transparency) channel for future use
        window_to_image_filter_[j]->ReadFrontBufferOff(); // read from the
        // back buffer
        // important for getting high update rate (If needed, images can be shown
        // with opencv)
        render_window_[j]->Render();

        //AddLightActors(scene_renderer_[j]);
    }

    //-------------------------------------------------
    // I added the third renderer to have a way of showing what is happening
    // to the other people who are not behind the console. this should be
    // integrated in the loop above later...
    scene_renderer_[2] = vtkSmartPointer<vtkOpenGLRenderer>::New();

    background_renderer_[2] = vtkSmartPointer<vtkOpenGLRenderer>::New();
    background_renderer_[2]->InteractiveOff();
    background_renderer_[2]->SetLayer(0);
    background_renderer_[2]->SetActiveCamera(cameras[0]->camera_real);

//    if(ar_mode_)
    scene_renderer_[2]->InteractiveOff();
    scene_renderer_[2]->SetLayer(1);

    cameras[2]->camera_virtual =
            scene_renderer_[2]->GetActiveCamera();

    if(num_render_windows_==1){
        background_renderer_[2]->SetViewport(view_port[2]);
        scene_renderer_[2]->SetViewport(view_port[2]);
    }


    scene_renderer_[2]->AddLight(lights[0]);
    scene_renderer_[2]->AddLight(lights[1]);

    if(with_shadows_)
        AddShadowPass(scene_renderer_[2]);

    render_window_[0]->AddRenderer(background_renderer_[2]);
    render_window_[0]->AddRenderer(scene_renderer_[2]);

    //------------------------------------------------

    if(num_render_windows_==1) {
        render_window_[0]->SetWindowName("Augmented Stereo");
        if(offScreen_rendering)
            render_window_[0]->SetOffScreenRendering(1);
    }
    else{
        render_window_[0]->SetWindowName("Augmented Left");
        render_window_[1]->SetWindowName("Augmented right");

        if(offScreen_rendering){
            render_window_[0]->SetOffScreenRendering(1);
            render_window_[1]->SetOffScreenRendering(1);
        }
    }

    // Set render window size if one window the width is double
    for (int j = 0; j < num_render_windows_; ++j) {
        render_window_[j]->SetSize((4-num_render_windows_) * 640, 480);
    }


    SetEnableBackgroundImage(true);

}


//------------------------------------------------------------------------------
Rendering::~Rendering()
{

    for (int j = 0; j < num_render_windows_; ++j) {
        render_window_[j]->RemoveRenderer(background_renderer_[j]);
        render_window_[j]->RemoveRenderer(scene_renderer_[j]);
    }


}




//------------------------------------------------------------------------------
void Rendering::SetEnableBackgroundImage(bool isEnabled)
{
    for (int i = 0; i < 2; ++i) {
        if (isEnabled)
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() == 0)
                background_renderer_[i]->AddActor(cameras[i]->image_actor_);
        }
        else
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() > 0)
                background_renderer_[i]->RemoveActor(cameras[i]->image_actor_);
        }
    }
    if (isEnabled)
    {
        if(background_renderer_[2]->GetActors()->GetNumberOfItems() == 0)
            background_renderer_[2]->AddActor(cameras[1]->image_actor_);
    }
    else
    {
        if(background_renderer_[2]->GetActors()->GetNumberOfItems() > 0)
            background_renderer_[2]->RemoveActor(cameras[1]->image_actor_);
    }
}





//------------------------------------------------------------------------------
void Rendering::UpdateCameraViewForActualWindowSize() {

    for (int i = 0; i <2; ++i) {

        int k = 0;
        if(num_render_windows_==2)
            k=i;
        int *window_size = render_window_[k]->GetActualSize();

        int single_win_size[2]
                = {window_size[0] / (4-num_render_windows_), window_size[1]};

        // update each windows view
        cameras[i]->UpdateVirtualView(single_win_size[0],
                                      single_win_size[1]);
        if(i==1)
            cameras[2]->UpdateVirtualView(single_win_size[0],
                                          single_win_size[1]);

        // update the background image for each camera
        cameras[i]->SetRealCameraToFaceImage(single_win_size);
    }
}



//------------------------------------------------------------------------------
void Rendering::AddActorToScene(vtkSmartPointer<vtkProp> actor) {

    scene_renderer_[0]->AddActor(actor);
    scene_renderer_[1]->AddActor(actor);
    scene_renderer_[2]->AddActor(actor);

}



void
Rendering::AddActorsToScene(std::vector<vtkSmartPointer<vtkProp> > actors) {


    vtkSmartPointer<vtkInformation> key_properties = vtkSmartPointer<vtkInformation>::New();
    key_properties->Set(vtkShadowMapBakerPass::OCCLUDER(),0);
    key_properties->Set(vtkShadowMapBakerPass::RECEIVER(),0);

    for (int i = 0; i <actors.size() ; ++i) {

        if(with_shadows_)
            actors[i]->SetPropertyKeys(key_properties);

        scene_renderer_[0]->AddViewProp(actors[i]);
        scene_renderer_[1]->AddViewProp(actors[i]);
        scene_renderer_[2]->AddViewProp(actors[i]);

    }

    scene_renderer_[0]->Modified();
    scene_renderer_[1]->Modified();
    scene_renderer_[2]->Modified();
}


//------------------------------------------------------------------------------
void Rendering::Render() {

//    scene_renderer_->Modified();
//    background_renderer_->Modified();
//    render_window_->Modified();
    // update  view angle (in case window changes size)
    if(ar_mode_)
        UpdateCameraViewForActualWindowSize();
    for (int i = 0; i < num_render_windows_; ++i) {
//    for (int i = 0; i < 1; ++i) {
        render_window_[i]->Render();
    }

}


//------------------------------------------------------------------------------
void Rendering::GetRenderedImage(cv::Mat *images) {

    // TODO: REWRITE FOR 2-WINDOW CASE (writes on the same image for now)

    for (int i = 0; i < num_render_windows_; ++i) {

        window_to_image_filter_[i]->Modified();
        vtkImageData *image = window_to_image_filter_[i]->GetOutput();
        window_to_image_filter_[i]->Update();

        // copy to cv Mat
        int dims[3];
        image->GetDimensions(dims);

//    std::cout << " dims[0] " << dims[0] << " dims[1] " << dims[1] << " "
//            "dims[2] " << dims[2] <<std::endl;
        if (dims[0] > 0) {
            cv::Mat openCVImage(dims[1], dims[0], CV_8UC3,
                                image->GetScalarPointer()); // Unsigned int, 4 channels
            // convert to bgr
            cv::cvtColor(openCVImage, images[i], cv::COLOR_RGB2BGR);

            // Flip because of different origins between vtk and OpenCV
            cv::flip(images[i], images[i], 0);
        }
    }
}

void Rendering::RemoveAllActorsFromScene() {

    scene_renderer_[0]->RemoveAllViewProps();
    scene_renderer_[1]->RemoveAllViewProps();
    scene_renderer_[2]->RemoveAllViewProps();
}

void Rendering::AddShadowPass(vtkSmartPointer<vtkOpenGLRenderer> renderer) {


    vtkSmartPointer<vtkCameraPass> cameraP =
            vtkSmartPointer<vtkCameraPass>::New();

    vtkSmartPointer<vtkOpaquePass> opaque =
            vtkSmartPointer<vtkOpaquePass>::New();

    vtkSmartPointer<vtkVolumetricPass> volume =
            vtkSmartPointer<vtkVolumetricPass>::New();
    vtkSmartPointer<vtkOverlayPass> overlay =
            vtkSmartPointer<vtkOverlayPass>::New();

    vtkSmartPointer<vtkLightsPass> lights_pass =
            vtkSmartPointer<vtkLightsPass>::New();

    vtkSmartPointer<vtkSequencePass> opaqueSequence =
            vtkSmartPointer<vtkSequencePass>::New();

    vtkSmartPointer<vtkRenderPassCollection> passes2 =
            vtkSmartPointer<vtkRenderPassCollection>::New();
    passes2->AddItem(lights_pass);
    passes2->AddItem(opaque);
    opaqueSequence->SetPasses(passes2);

    vtkSmartPointer<vtkCameraPass> opaqueCameraPass =
            vtkSmartPointer<vtkCameraPass>::New();
    opaqueCameraPass->SetDelegatePass(opaqueSequence);

    vtkSmartPointer<vtkShadowMapBakerPass> shadowsBaker =
            vtkSmartPointer<vtkShadowMapBakerPass>::New();
    shadowsBaker->SetOpaquePass(opaqueCameraPass);
    shadowsBaker->SetResolution(2048);
    // To cancel self-shadowing.
    shadowsBaker->SetPolygonOffsetFactor(2.4f);
    shadowsBaker->SetPolygonOffsetUnits(5.0f);

    vtkSmartPointer<vtkShadowMapPass> shadows =
            vtkSmartPointer<vtkShadowMapPass>::New();
    shadows->SetShadowMapBakerPass(shadowsBaker);
    shadows->SetOpaquePass(opaqueSequence);

    vtkSmartPointer<vtkSequencePass> seq =
            vtkSmartPointer<vtkSequencePass>::New();
    vtkSmartPointer<vtkRenderPassCollection> passes =
            vtkSmartPointer<vtkRenderPassCollection>::New();
    passes->AddItem(shadowsBaker);
    passes->AddItem(shadows);
    passes->AddItem(lights_pass);
    passes->AddItem(volume);
    passes->AddItem(overlay);
    seq->SetPasses(passes);
    cameraP->SetDelegatePass(seq);

    renderer->SetPass(cameraP);

}

void Rendering::ToggleFullScreen() {

    for (int k = 0; k < num_render_windows_; ++k) {
        if(render_window_[k]->GetFullScreen()==1)
            render_window_[k]->SetFullScreen(0);
        else
            render_window_[k]->SetFullScreen(1);
    }
}

bool Rendering::AreImagesNew() {

    return (cameras[0]->IsImageNew() && cameras[1]->IsImageNew());

}


// For each spotlight, add a light frustum wireframe representation and a cone
// wireframe representation, colored with the light color.
void AddLightActors(vtkRenderer *r)
{
    assert("pre: r_exists" && r!=0);

    vtkLightCollection *lights=r->GetLights();

    lights->InitTraversal();
    vtkLight *l=lights->GetNextItem();
    while(l!=0)
    {
        double angle=l->GetConeAngle();
        if(l->LightTypeIsSceneLight() && l->GetPositional()
           && angle<180.0) // spotlight
        {
            vtkLightActor *la=vtkLightActor::New();
            la->SetLight(l);
            r->AddViewProp(la);
            la->Delete();
        }
        l=lights->GetNextItem();
    }
}
