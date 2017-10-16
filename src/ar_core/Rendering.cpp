//
// Created by nima on 4/12/17.
//
#include "Rendering.h"
#include "VTKConversions.h"


// helper function for debugging light related issues
void AddLightActors(vtkRenderer *r);


Rendering::Rendering(bool AR_mode, uint num_windows, bool with_shaodws,
                     bool offScreen_rendering,
                     std::vector<int> window_position)
        : num_render_windows_(num_windows),
          with_shadows_(with_shaodws),
          ar_mode_(AR_mode)
{
    // make sure the number of windows are alright
    if(num_render_windows_ <1) num_render_windows_ =1;
    else if(num_render_windows_ >2) num_render_windows_ = 2;

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

        image_importer_[i] = vtkSmartPointer<vtkImageImport>::New();
        image_actor_[i] = vtkSmartPointer<vtkImageActor>::New();
        camera_image_[i] = vtkSmartPointer<vtkImageData>::New();

        scene_camera_[i] = new CalibratedCamera;
        scene_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();

        // In AR the background renderer shows the real camera images from
        // the world
        background_renderer_[i] = vtkSmartPointer<vtkOpenGLRenderer>::New();
        background_renderer_[i]->InteractiveOff();
        background_renderer_[i]->SetLayer(0);

        background_camera_[i] = new CalibratedCamera;
        background_renderer_[i]->SetActiveCamera(background_camera_[i]->camera);

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
        scene_camera_[i]->camera = scene_renderer_[i]->GetActiveCamera();

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
    background_renderer_[2]->SetActiveCamera(background_camera_[0]->camera);

//    if(ar_mode_)
    scene_renderer_[2]->InteractiveOff();
    scene_renderer_[2]->SetLayer(1);


    scene_camera_[2] = new CalibratedCamera;
    scene_camera_[2]->camera = scene_renderer_[2]->GetActiveCamera();

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
void Rendering::SetWorldToCameraTransform(const cv::Vec3d cam_rvec[],
                                          const cv::Vec3d cam_tvec[]) {


    for (int k = 0; k < 2; ++k) {

        cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
        cv::Rodrigues(cam_rvec[k], rotationMatrix);

        vtkSmartPointer<vtkMatrix4x4>
                world_to_camera_transform =
                vtkSmartPointer<vtkMatrix4x4>::New();
        world_to_camera_transform->Identity();

        // Convert to VTK matrix.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                world_to_camera_transform->SetElement(
                        i, j, rotationMatrix.at<double>(
                                i, j
                        ));
            }
            world_to_camera_transform->SetElement(i, 3, cam_tvec[k][i]);
        }

        vtkSmartPointer<vtkMatrix4x4> camera_to_world_transform =
                vtkSmartPointer<vtkMatrix4x4>::New();
        camera_to_world_transform->Identity();

        camera_to_world_transform->DeepCopy(world_to_camera_transform);

        camera_to_world_transform->Invert();

        scene_camera_[k]->SetExtrinsicParameters(camera_to_world_transform);
        if(k==1)
            scene_camera_[2]->SetExtrinsicParameters(camera_to_world_transform);


    }


    //lights[0]->SetPosition(
    //    camera_to_world_transform_[0]->Element[0][3],
    //    camera_to_world_transform_[0]->Element[1][3],
    //    camera_to_world_transform_[0]->Element[2][3]
    //);


}

//------------------------------------------------------------------------------
void Rendering::SetEnableBackgroundImage(bool isEnabled)
{
    for (int i = 0; i < 2; ++i) {
        if (isEnabled)
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() == 0)
                background_renderer_[i]->AddActor(image_actor_[i]);
        }
        else
        {
            if(background_renderer_[i]->GetActors()->GetNumberOfItems() > 0)
                background_renderer_[i]->RemoveActor(image_actor_[i]);
        }
    }
    if (isEnabled)
    {
        if(background_renderer_[2]->GetActors()->GetNumberOfItems() == 0)
            background_renderer_[2]->AddActor(image_actor_[1]);
    }
    else
    {
        if(background_renderer_[2]->GetActors()->GetNumberOfItems() > 0)
            background_renderer_[2]->RemoveActor(image_actor_[1]);
    }
}


//------------------------------------------------------------------------------
void Rendering::SetCameraIntrinsics(const cv::Mat intrinsics[])
{
    for (int i = 0; i < 2; ++i) {

        background_camera_[i]->SetIntrinsicParameters(intrinsics[i].at<double>(0, 0),
                                                      intrinsics[i].at<double>(1, 1),
                                                      intrinsics[i].at<double>(0, 2),
                                                      intrinsics[i].at<double>(1, 2));
        scene_camera_[i]->SetIntrinsicParameters(intrinsics[i].at<double>(0, 0),
                                                 intrinsics[i].at<double>(1, 1),
                                                 intrinsics[i].at<double>(0, 2),
                                                 intrinsics[i].at<double>(1, 2));
    }
    scene_camera_[2]->SetIntrinsicParameters(intrinsics[1].at<double>(0, 0),
                                             intrinsics[1].at<double>(1, 1),
                                             intrinsics[1].at<double>(0, 2),
                                             intrinsics[1].at<double>(1, 2));
}


//------------------------------------------------------------------------------
void
Rendering::SetImageCameraToFaceImage(const int id, const int *window_size) {

    int imageSize[3];
    image_importer_[id]->GetOutput()->GetDimensions(imageSize);

    double spacing[3];
    image_importer_[id]->GetOutput()->GetSpacing(spacing);

    double origin[3];
    image_importer_[id]->GetOutput()->GetOrigin(origin);

    background_camera_[id]->SetCemraToFaceImage(window_size, imageSize,
                                                spacing, origin);

}


//------------------------------------------------------------------------------
void Rendering::UpdateBackgroundImage(cv::Mat  img[]) {

    for (int i = 0; i < 2; ++i) {
//    cv::flip(src, _src, 0);
        cv::cvtColor(img[i], img[i], cv::COLOR_BGR2RGB);
        image_importer_[i]->SetImportVoidPointer( img[i].data );
        image_importer_[i]->Modified();
        image_importer_[i]->Update();
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
        scene_camera_[i]->UpdateView(single_win_size[0],
                                     single_win_size[1]);
        if(i==1)
            scene_camera_[2]->UpdateView(single_win_size[0],
                                         single_win_size[1]);

        // update the background image for each camera
        SetImageCameraToFaceImage(i, single_win_size);
    }
}


//------------------------------------------------------------------------------
void Rendering::ConfigureBackgroundImage(cv::Mat *img) {

    int image_width = img[0].size().width;
    int image_height = img[0].size().height;

    for (int i = 0; i < 2; ++i) {
        assert( img[i].data != NULL );

        scene_camera_[i]->SetCameraImageSize(image_width, image_height);
        background_camera_[i]->SetCameraImageSize(image_width, image_height);

        if (camera_image_[i]) {
            image_importer_[i]->SetOutput(camera_image_[i]);
        }
        image_importer_[i]->SetDataSpacing(1, 1, 1);
        image_importer_[i]->SetDataOrigin(0, 0, 0);
        image_importer_[i]->SetWholeExtent(0, image_width - 1, 0,
                                           image_height - 1, 0, 0);
        image_importer_[i]->SetDataExtentToWholeExtent();
        image_importer_[i]->SetDataScalarTypeToUnsignedChar();
        image_importer_[i]->SetNumberOfScalarComponents(img[i].channels());
        image_importer_[i]->SetImportVoidPointer(img[i].data);
        image_importer_[i]->Update();

        image_actor_[i]->SetInputData(camera_image_[i]);
    }
    scene_camera_[2]->SetCameraImageSize(image_width, image_height);

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
