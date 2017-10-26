//
// Created by nima on 4/12/17.
//
#include <custom_conversions/Conversions.h>
#include "Rendering.h"
#include "VTKConversions.h"


// helper function for debugging light related issues
void AddLightActors(vtkRenderer *r);


Rendering::Rendering(ros::NodeHandle *n)
{

    n->param<bool>("AR_mode", ar_mode_, false);
    ROS_INFO("AR mode: %s", ar_mode_ ? "true" : "false");

    n->param<int>("one_window_mode", num_render_windows_, false);
    ROS_INFO("Rendered Images will be shown in %i window(s): ",
             num_render_windows_);

    n->param<bool>("with_shadows", with_shadows_, false);
    ROS_INFO("Shadows Generation: %s", with_shadows_ ? "true" : "false");

    bool offScreen_rendering;
    n->param<bool>("offScreen_rendering", offScreen_rendering, false);
    ROS_INFO("offScreen_rendering: %s", offScreen_rendering ? "true" : "false");


    // make sure the number of windows are alright
    if(num_render_windows_ <1) num_render_windows_ =1;
    else if(num_render_windows_ >2) num_render_windows_ = 2;

    if(ar_mode_)
        it = new image_transport::ImageTransport(*n);

    std::string left_cam_name;
    if (n->getParam("left_cam_name", left_cam_name))
        cameras[0] = new ARCamera(n, it, left_cam_name);
    else {
        cameras[0] = new ARCamera(n);
        std::vector<double> temp_vec = {0.057, -0.022, 0.290, 0.0271128721729,
                                        0.87903000839, -0.472201765689, 0.0599719016889};
        KDL::Frame temp_frame;
        conversions::VectorToKDLFrame(temp_vec, temp_frame);
        cameras[0]->SetWorldToCamTf(temp_frame);
    }
    std::string right_cam_name;
    if (n->getParam("right_cam_name", right_cam_name)) {
        cameras[1] = new ARCamera(n, it, right_cam_name);
        cameras[2] = new ARCamera(n, it, right_cam_name);
    }
    else {
        cameras[1] = new ARCamera(n);
        cameras[2] = new ARCamera(n);
        std::vector<double> temp_vec = {0.057, -0.022, 0.290, 0.0271128721729,
                                        0.87903000839, -0.472201765689, 0.0599719016889};
        KDL::Frame temp_frame;
        conversions::VectorToKDLFrame(temp_vec, temp_frame);
        cameras[0]->SetWorldToCamTf(temp_frame);
        cameras[2]->SetWorldToCamTf(temp_frame);
    }

    render_window_[0] = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_[0]->BordersOff();

    std::vector<int> window_position(4, 0);
    n->getParam("window_position", window_position);
    if(num_render_windows_==1)
        render_window_[0]->SetPosition(window_position[0], window_position[1]);
    else if(num_render_windows_==2) {
        render_window_[1] = vtkSmartPointer<vtkRenderWindow>::New();
        render_window_[1]->SetPosition(window_position[2], window_position[3]);
        render_window_[1]->BordersOff();
    }

    SetupLights();

    double view_port[3][4] = {{0.333, 0.0, 0.666, 1.0}
            , {0.666, 0.0, 1.0, 1.0}
            ,{0.0, 0.0, 0.333, 1.0}};

    bool publish_overlayed_images;
    n->param<bool>("publish_overlayed_images", publish_overlayed_images, false);
    ROS_INFO("Rendered Images will be grabbed from gpu and published: %s",
             publish_overlayed_images ? "true" : "false");

    for (int i = 0; i < 3; ++i) {


        scene_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();

        // In AR the background renderer shows the real camera images from
        // the world
        if(ar_mode_) {
            background_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
            background_renderer_[i]->InteractiveOff();
            background_renderer_[i]->SetLayer(0);
            background_renderer_[i]->SetActiveCamera(cameras[i]->camera_real);

        }


        // in AR we do not need interactive windows or dual layer render
        scene_renderer_[i]->InteractiveOff();
        scene_renderer_[i]->SetLayer((int)ar_mode_);
//        scene_renderer_[i]->SetLayer(0);


        if(num_render_windows_==1){
            if(ar_mode_)
                background_renderer_[i]->SetViewport(view_port[i]);
            scene_renderer_[i]->SetViewport(view_port[i]);
        }

        scene_renderer_[i]->AddLight(lights[0]);
        scene_renderer_[i]->AddLight(lights[1]);

        //scene_renderer_[i]->ResetCamera();
//        cameras[i]->camera_virtual= scene_renderer_[i]->GetActiveCamera();
        scene_renderer_[i]->SetActiveCamera(cameras[i]->camera_virtual);

        if(with_shadows_)
            AddShadowPass(scene_renderer_[i]);

        int j=0;
        if(num_render_windows_==2)
            j=i;

        render_window_[j]->SetNumberOfLayers(2);
        if(ar_mode_)
            render_window_[j]->AddRenderer(background_renderer_[i]);

        render_window_[j]->AddRenderer(scene_renderer_[i]);

        if(publish_overlayed_images) {
            window_to_image_filter_[j] =
                    vtkSmartPointer<vtkWindowToImageFilter>::New();
            window_to_image_filter_[j]->SetInput(render_window_[j]);
            //    window_to_image_filter_->SetInputBufferTypeToRGBA(); //record  he
            // alpha (transparency) channel for future use
            window_to_image_filter_[j]->ReadFrontBufferOff(); // read from
            // the back buffer important for getting high update rate (If
            // needed, images can be shown with opencv)
        }

        render_window_[j]->Render();

        //AddLightActors(scene_renderer_[j]);
    }

    //-------------------------------------------------
    // I added the third renderer to have a way of showing what is happening
    // to the other people who are not behind the console. this should be
    // integrated in the loop above later...

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


    if(ar_mode_)
        SetEnableBackgroundImage(true);


}


//------------------------------------------------------------------------------
Rendering::~Rendering()
{

    for (int j = 0; j < num_render_windows_; ++j) {
        if(ar_mode_)
            render_window_[j]->RemoveRenderer(background_renderer_[j]);
        render_window_[j]->RemoveRenderer(scene_renderer_[j]);
    }

    delete cameras[0];
    delete cameras[1];
    delete cameras[2];

    if(ar_mode_)
        delete it;

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
            background_renderer_[2]->AddActor(cameras[2]->image_actor_);
    }
    else
    {
        if(background_renderer_[2]->GetActors()->GetNumberOfItems() > 0)
            background_renderer_[2]->RemoveActor(cameras[2]->image_actor_);
    }
}





//------------------------------------------------------------------------------
void Rendering::UpdateCameraViewForActualWindowSize() {

    for (int i = 0; i <3; ++i) {

        int k = 0;
        if(num_render_windows_==2)
            k=i;
        int *window_size = render_window_[k]->GetActualSize();

        int single_win_size[2]
                = {window_size[0] / (4-num_render_windows_), window_size[1]};

        // update each windows view
        cameras[i]->UpdateVirtualView(single_win_size[0],
                                      single_win_size[1]);

        // update the background image for each camera
        cameras[i]->UpdateBackgroundImage(single_win_size);
    }
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

void Rendering::SetupLights() {

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
