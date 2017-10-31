//
// Created by nima on 4/12/17.
//
#include <custom_conversions/Conversions.h>
#include "Rendering.h"


// helper function for debugging light related issues
void AddLightActors(vtkRenderer *r);


Rendering::Rendering(ros::NodeHandlePtr n, const bool ar_mode,
                     const int n_views, const bool one_window_per_view)
        :
        ar_mode_(ar_mode), num_views(n_views), it(NULL)
{

    num_windows = int(one_window_per_view)*num_views + int(!one_window_per_view);

    std::string cam_names[num_views];

    if(ar_mode_){
        GetCameraNames( n, num_views, cam_names);
        it = new image_transport::ImageTransport(*n);
    }

    for (int k = 0; k < num_views; ++k) {
        if(ar_mode_)
            cameras[k] = new ARCamera(n, it, cam_names[k]);
        else {
            cameras[k] = new ARCamera(n);
        }
    }

    n->param<bool>("with_shadows", with_shadows_, false);
    ROS_INFO("Shadows Generation: %s", with_shadows_ ? "true" : "false");
    bool offScreen_rendering;
    n->param<bool>("offScreen_rendering", offScreen_rendering, false);
    ROS_INFO("offScreen_rendering: %s", offScreen_rendering ? "true" : "false");

    n->param<bool>("publish_overlayed_images", publish_overlayed_images_, false);
    ROS_INFO("Rendered Images will be grabbed from gpu and published: %s",
             publish_overlayed_images_ ? "true" : "false");

    std::vector<int> window_positions(6, 0);
    n->getParam("window_positions", window_positions);
    std::vector<int> view_resolution(2, 640);
    view_resolution[1] = 480;
    n->getParam("view_resolution", view_resolution);

    SetupLights();

    double view_port[2][3][4] = {
            {{0. , 0., 1./2., 1.}, {1./2., 0., 1.   , 1.}, {0, 0, 0, 0} },
            {{0. , 0., 1./3., 1.}, {1./3., 0., 2./3., 1.},{2./3., 0., 1., 1.}}
    }; // first dimension is 2-view or 3-view case

    for (int i = 0; i < num_views; ++i) {

        scene_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
        // in AR we do not need interactive windows or dual layer render
        scene_renderer_[i]->InteractiveOff();
        scene_renderer_[i]->SetLayer((int)ar_mode_);

        // In AR the background renderer shows the real camera images from
        // the world
        if(ar_mode_) {
            background_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
            background_renderer_[i]->InteractiveOff();
            background_renderer_[i]->SetLayer(0);
            background_renderer_[i]->SetActiveCamera(cameras[i]->camera_real);
        }

        if(with_shadows_)
            AddShadowPass(scene_renderer_[i]);

        scene_renderer_[i]->AddLight(lights[0]);
        scene_renderer_[i]->AddLight(lights[1]);
        scene_renderer_[i]->SetActiveCamera(cameras[i]->camera_virtual);

        if(num_views>num_windows){ // when there are multiple views in a window
            if(ar_mode_)
                background_renderer_[i]->SetViewport(view_port[num_views-2][i]);
            scene_renderer_[i]->SetViewport(view_port[num_views-2][i]);
        }

        int j=0;
        if(num_windows>1)
            j=i;
        if(i<num_windows) {
            render_window_[j] = vtkSmartPointer<vtkRenderWindow>::New();
//            render_window_[j]->BordersOff();
            render_window_[j]->SetPosition(window_positions[2 * j],
                                           window_positions[2 * j + 1]);
            render_window_[j]->SetNumberOfLayers(2);
            render_window_[j]->SetWindowName("Augmented Stereo");
            if(offScreen_rendering)
                render_window_[j]->SetOffScreenRendering(1);
            render_window_[j]->SetSize(num_views * view_resolution[0],
                                       view_resolution[1]);
        }
        render_window_[j]->AddRenderer(scene_renderer_[i]);
        if(ar_mode_)
            render_window_[j]->AddRenderer(background_renderer_[i]);

        if(publish_overlayed_images_) {
            window_to_image_filter_[j] =
                    vtkSmartPointer<vtkWindowToImageFilter>::New();
            window_to_image_filter_[j]->SetInput(render_window_[j]);
            //    window_to_image_filter_->SetInputBufferTypeToRGBA(); //record  he
            // alpha (transparency) channel for future use
            window_to_image_filter_[j]->ReadFrontBufferOff(); // read from
            // the back buffer important for getting high update rate (If
            // needed, images can be shown with opencv)
            cvNamedWindow("Augmented Stereo", CV_WINDOW_NORMAL);
            publisher_stereo_overlayed = it->advertise("stereo/image_color", 1);
        }



        render_window_[j]->Render();

        //AddLightActors(scene_renderer_[j]);
    }


    if(ar_mode_)
        SetEnableBackgroundImage(true);

    Render();

}


//------------------------------------------------------------------------------
Rendering::~Rendering()
{
    for (int j = 0; j < num_windows; ++j) {
        if(ar_mode_)
            render_window_[j]->RemoveRenderer(background_renderer_[j]);
        render_window_[j]->RemoveRenderer(scene_renderer_[j]);
    }

    for (int i = 0; i < num_views; ++i) {
        delete cameras[i];
    }

    if(it)
        delete it;
}


//------------------------------------------------------------------------------
void Rendering::SetEnableBackgroundImage(bool isEnabled)
{
    for (int i = 0; i < num_views; ++i) {
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
}


//------------------------------------------------------------------------------
void Rendering::UpdateCameraViewForActualWindowSize() {

    for (int i = 0; i <num_views; ++i) {
        int k = 0;
        if(num_windows>1)
            k=i;
        int *window_size = render_window_[k]->GetActualSize();

        int view_size[2] = {window_size[0] / num_views, window_size[1]};

        cameras[i]->RefreshCamera(view_size);
    }
}

void
Rendering::AddActorsToScene(std::vector<vtkSmartPointer<vtkProp> > actors) {

    vtkSmartPointer<vtkInformation> key_properties = vtkSmartPointer<vtkInformation>::New();
    key_properties->Set(vtkShadowMapBakerPass::OCCLUDER(),0);
    key_properties->Set(vtkShadowMapBakerPass::RECEIVER(),0);

    for (int j = 0; j < num_views; ++j) {
        for (int i = 0; i <actors.size() ; ++i) {

            if(with_shadows_)
                actors[i]->SetPropertyKeys(key_properties);
            scene_renderer_[j]->AddViewProp(actors[i]);
        }
        scene_renderer_[j]->Modified();
    }

}


//------------------------------------------------------------------------------
void Rendering::Render() {

    // update  view angle (in case window changes size)
    if(ar_mode_)
        UpdateCameraViewForActualWindowSize();
    for (int i = 0; i < num_windows; ++i) {
        render_window_[i]->Render();
    }

    // Copy the rendered image to memory, show it and/or publish it.
    if(publish_overlayed_images_)
        PublishRenderedImages();
}


//------------------------------------------------------------------------------
void Rendering::GetRenderedImage(cv::Mat *images) {

    // TODO: REWRITE FOR 2-WINDOW CASE (writes on the same image for now)

    for (int i = 0; i < num_windows; ++i) {

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

    for (int i = 0; i < num_views; ++i) {
        scene_renderer_[i]->RemoveAllViewProps();
    }

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

    for (int k = 0; k < num_windows; ++k) {
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


void Rendering::GetCameraNames(ros::NodeHandlePtr n, const int num_views,
                               std::string cam_names[]) {

    std::stringstream param_name;
    int name_count = 0;
    for (int k = 0; k < num_views; ++k) {
        param_name << "cam_" << k << "_name" << std::endl;
        std::cout << "aaaaaaaaaaaaaaa " << param_name.str()<< std::endl;
        std::string yep;
        if (n->getParam("cam_0_name", cam_names[k]))
            name_count+=1;
    }

    if(name_count!=num_views)
        throw std::runtime_error("Not enough cam names provided.");
}

// -----------------------------------------------------------------------------
void Rendering::PublishRenderedImages() {

    // todo: Update the implementation of this
    cv::Mat augmented_images[2];

    char key = (char)cv::waitKey(1);
    if (key == 27) // Esc
        ros::shutdown();
    else if (key == 'f')  //full screen
//        SwitchFullScreenCV(cv_window_names[0]);

        GetRenderedImage(augmented_images);
//    if(one_window_mode){
    cv::imshow("Augmented Stereo", augmented_images[0]);
    publisher_stereo_overlayed.publish(
            cv_bridge::CvImage(std_msgs::Header(),
                               "bgr8", augmented_images[0]).toImageMsg());
//    }
//    else{
//        for (int i = 0; i < 2; ++i) {
//            cv::imshow(cv_window_names[i], augmented_images[i]);
//            publisher_overlayed[i].publish(
//                    cv_bridge::CvImage(std_msgs::Header(), "bgr8",
//                                       augmented_images[i]).toImageMsg());
//        }
//        if (key == 'f')  //full screen
//            SwitchFullScreenCV(cv_window_names[1]);
//    }

}

void Rendering::SetManipulatorInterestedInCamPose(Manipulator * in) {
    cameras[0]->SetPtrManipulatorInterestedInCamPose(in);
}

void Rendering::SetMainCameraPose(const KDL::Frame &pose) {
    for (int i = 0; i < num_views; ++i) {
        cameras[i]->SetWorldToCamTf(pose);
    }
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


