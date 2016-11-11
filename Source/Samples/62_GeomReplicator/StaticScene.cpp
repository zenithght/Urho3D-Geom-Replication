//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Container/ArrayPtr.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include <stdio.h>

#include "StaticScene.h"

#include <Urho3D/DebugNew.h>

//=============================================================================
//=============================================================================
#define ONE_SEC_DURATION 1000

//=============================================================================
//=============================================================================
unsigned GeomReplicator::Replicate(const PODVector<PRotScale> &qplist)
{
    Model *pModel = GetModel();
    Geometry *pGeometry = pModel->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    IndexBuffer *pIbuffer = pGeometry->GetIndexBuffer();
    unsigned uElementMask = pVbuffer->GetElementMask();
    unsigned vertexSize = pVbuffer->GetVertexSize();
    unsigned numVertices = pVbuffer->GetVertexCount();

    // sizes
    unsigned origVertsBuffSize = vertexSize * numVertices;
    unsigned numIndeces = pIbuffer->GetIndexCount();
    unsigned origIdxBuffSize = numIndeces * sizeof(unsigned short);

    PrimitiveType ptype = pGeometry->GetPrimitiveType();
    BoundingBox bbox;

    // cpy the orig buffers
    SharedArrayPtr<unsigned char> origVertBuff( new unsigned char[origVertsBuffSize] );
    SharedArrayPtr<unsigned short> origIdxBuff( new unsigned short[numIndeces] );

    const unsigned char *pVertexData = (const unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());

    // copy orig buffer
    if (pVertexData)
    {
        memcpy(origVertBuff.Get(), pVertexData, origVertsBuffSize);
        pVbuffer->Unlock();
    }
    void *pIndexData = (void*)pIbuffer->Lock(0, pIbuffer->GetIndexCount());

    if ( pIndexData )
    {
        memcpy(origIdxBuff.Get(), pIndexData, origIdxBuffSize);
        pIbuffer->Unlock();
    }

    // find the max index value
    unsigned maxIdxVal = 0;
    for (unsigned i = 0; i < numIndeces; ++i )
    {
        if ( origIdxBuff[ i ] > maxIdxVal )
        {
            maxIdxVal = origIdxBuff[ i ];
        }
    }
    // count for index 0
    maxIdxVal++;

    // create new indeces for the replicated size
    PODVector<unsigned short> newIndexList;
    PODVector<int> newIntIndexList;

    for ( unsigned i = 0; i < qplist.Size(); ++i )
    {
        for (unsigned j = 0; j < numIndeces; ++j)
        {
            unsigned short idx = origIdxBuff[ j ];
            newIndexList.Push( i*maxIdxVal + origIdxBuff[ j ] );
            newIntIndexList.Push( i*maxIdxVal + origIdxBuff[ j ] );
        }
    }
    bool isOver64k = newIndexList.Size() > 1024 * 64;
    pIbuffer->SetSize(newIndexList.Size(), isOver64k );

    pIndexData = (void*)pIbuffer->Lock(0, pIbuffer->GetIndexCount());

    if ( pIndexData )
    {
        if ( isOver64k )
        {
            memcpy(pIndexData, &newIntIndexList[0], sizeof(int) * newIntIndexList.Size());
        }
        else
        {
            memcpy(pIndexData, &newIndexList[0], sizeof(unsigned short) * newIndexList.Size());
        }

        pIbuffer->Unlock();
    }

    // dupe verts
    pVbuffer->SetSize( numVertices * qplist.Size(), uElementMask );
    unsigned nsize = pVbuffer->GetVertexCount();
    unsigned nvertexSize = pVbuffer->GetVertexSize();

    pVertexData = (unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());

    if ( pVertexData )
    {
        for ( unsigned i = 0; i < qplist.Size(); ++i )
        {

            for ( unsigned j = 0; j < numVertices; ++j )
            {
                unsigned char *pOrigDataAlign = (unsigned char *)(origVertBuff.Get() + j * vertexSize);
                unsigned char *pDataAlign = (unsigned char *)(pVertexData + (i * numVertices + j) * vertexSize);

                Vector3 pos = qplist[i].pos;
                Quaternion rot(qplist[i].rot);
                Matrix3x4 mat( pos, rot, qplist[i].scale);

                // position
                const Vector3 vPos = *reinterpret_cast<Vector3*>( pOrigDataAlign );
                Vector3 &nPos = *reinterpret_cast<Vector3*>( pDataAlign );
                nPos = mat * vPos;

                bbox.Merge(nPos);

                // normal
                const Vector3 vNorm = *reinterpret_cast<Vector3*>( pOrigDataAlign + sizeof(Vector3) );
                Vector3 &norm = *reinterpret_cast<Vector3*>( pDataAlign + sizeof(Vector3) );
                norm = rot * vNorm;
                
                // how about tangent?
                
                // copy everything else excluding the position and normal
                memcpy(pDataAlign + 2*sizeof(Vector3), pOrigDataAlign + 2*sizeof(Vector3), vertexSize - 2*sizeof(Vector3) );
            }
        }

        //unlock
        pVbuffer->Unlock();
   }

    // set draw range and bounding box
    pGeometry->SetDrawRange(TRIANGLE_LIST, 0, newIndexList.Size());

    SetBoundingBox( bbox );

    return qplist.Size();
}

//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(StaticScene)

StaticScene::StaticScene(Context* context) :
    Sample(context)
    , framesCount_(0)
    , timeToLoad_(0)
{
    GeomReplicator::RegisterObject(context);
}

void StaticScene::Setup()
{
    engineParameters_["WindowTitle"]  = GetTypeName();
    engineParameters_["LogName"]      = GetSubsystem<FileSystem>()->GetAppPreferencesDir("urho3d", "logs") + GetTypeName() + ".log";
    engineParameters_["FullScreen"]   = false;
    engineParameters_["Headless"]     = false;
    engineParameters_["WindowWidth"]  = 1280; 
    engineParameters_["WindowHeight"] = 720;
}

void StaticScene::Start()
{
    Sample::Start();

    CreateScene();

    CreateStatusText();

    SetupViewport();

    SubscribeToEvents();

    Sample::InitMouseMode(MM_RELATIVE);
}

void StaticScene::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    fpsTimer_.Reset();

    scene_ = new Scene(context_);

    scene_->CreateComponent<Octree>();

    Node* planeNode = scene_->CreateChild("Plane");
    planeNode->SetScale(Vector3(100.0f, 1.0f, 100.0f));
    StaticModel* planeObject = planeNode->CreateComponent<StaticModel>();
    planeObject->SetModel(cache->GetResource<Model>("Models/Plane.mdl"));
    planeObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));

    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f)); 
    Light* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);

    // load nodes or create replication mesh
    bool loadNodes = false;
    const unsigned NUM_OBJECTS = 10000;

    for (unsigned i = 0; i < NUM_OBJECTS; ++i)
    {
        PRotScale qp;
        qp.pos = Vector3(Random(90.0f) - 45.0f, 0.0f, Random(90.0f) - 45.0f);
        qp.rot = Quaternion(0.0f, Random(360.0f), 0.0f);
        qp.scale = 0.5f + Random(2.0f);
        qpList_.Push(qp);

        if ( loadNodes )
        {
            Node* mushroomNode = scene_->CreateChild("Vegbrush");
            mushroomNode->SetPosition(qp.pos);
            mushroomNode->SetRotation(qp.rot);
            mushroomNode->SetScale(qp.scale);
            StaticModel* mushroomObject = mushroomNode->CreateComponent<StaticModel>();
            mushroomObject->SetModel(cache->GetResource<Model>("Models/Veg/vegbrush.mdl"));
            mushroomObject->SetMaterial(cache->GetResource<Material>("Models/Veg/veg-alphamask.xml"));
        }
    }

    if ( !loadNodes )
    {
        Model *pModel = cache->GetResource<Model>("Models/Veg/vegbrush.mdl");
        SharedPtr<Model> cloneModel = pModel->Clone();

        nodeRep_ = scene_->CreateChild("Vegrep");
        vegReplicator_ = nodeRep_->CreateComponent<GeomReplicator>();
        vegReplicator_->SetModel( cloneModel );
        vegReplicator_->SetMaterial(cache->GetResource<Material>("Models/Veg/veg-alphamask.xml"));
        vegReplicator_->Replicate(qpList_);
    }

    // camera
    cameraNode_ = scene_->CreateChild("Camera");
    cameraNode_->CreateComponent<Camera>();

    // Set an initial position for the camera scene node above the plane
    cameraNode_->SetPosition(Vector3(-4.0f, 3.0f, -50.0f));

    timeToLoad_ = fpsTimer_.GetMSec(true);
}

void StaticScene::CreateStatusText()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    textStatus_ = ui->GetRoot()->CreateChild<Text>();
    textStatus_->SetText("");
    textStatus_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 10);
    textStatus_->SetPosition(10, 10);
    textStatus_->SetColor(Color::YELLOW);
}

void StaticScene::SetupViewport()
{
    Renderer* renderer = GetSubsystem<Renderer>();

    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void StaticScene::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    Input* input = GetSubsystem<Input>();

    const float MOVE_SPEED = 20.0f;
    const float MOUSE_SENSITIVITY = 0.1f;

    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
}

void StaticScene::SubscribeToEvents()
{
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(StaticScene, HandleUpdate));
}

void StaticScene::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    float timeStep = eventData[P_TIMESTEP].GetFloat();

    MoveCamera(timeStep);

    framesCount_++;
    if ( fpsTimer_.GetMSec(false) >= ONE_SEC_DURATION )
    {
        Renderer *renderer = GetSubsystem<Renderer>();
        String stat, x, y, z;
        char buff[20];
        sprintf(buff, ", cam: %.1f, ", cameraNode_->GetPosition().x_);
        x = String(buff);
        sprintf(buff, "%.1f, ", cameraNode_->GetPosition().y_);
        y = String(buff);
        sprintf(buff, "%.1f", cameraNode_->GetPosition().z_);
        z = String(buff);

        stat.AppendWithFormat( "tris: %d, batches: %d, fps: %d, load time=%d", 
                               renderer->GetNumPrimitives(),
                               renderer->GetNumBatches(),
                               framesCount_,
                               timeToLoad_);
        //stat += x + y + z;
        textStatus_->SetText(stat);
        framesCount_ = 0;
        fpsTimer_.Reset();
    }

}
