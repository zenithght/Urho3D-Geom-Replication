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
unsigned GeomReplicator::Replicate(const PODVector<PRotScale> &qplist, const Vector3 &normal)
{
    Geometry *pGeometry = GetModel()->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    IndexBuffer *pIbuffer = pGeometry->GetIndexBuffer();
    unsigned uElementMask = pVbuffer->GetElementMask();
    unsigned vertexSize = pVbuffer->GetVertexSize();
    unsigned numVertices = pVbuffer->GetVertexCount();

    // for movement
    numVertsPerGeom = numVertices;

    // retain bbox as the size grows
    BoundingBox bbox;

    // cpy orig vbuffs
    unsigned origVertsBuffSize = vertexSize * numVertices;
    SharedArrayPtr<unsigned char> origVertBuff( new unsigned char[origVertsBuffSize] );
    const unsigned char *pVertexData = (const unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());

    if (pVertexData)
    {
        memcpy(origVertBuff.Get(), pVertexData, origVertsBuffSize);
        pVbuffer->Unlock();
    }

    // replicate
    pVbuffer->SetSize( numVertices * qplist.Size(), uElementMask );
    pVertexData = (unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());
    bool overrideNormal = normal != Vector3::ZERO;

    if ( pVertexData )
    {
        for ( unsigned i = 0; i < qplist.Size(); ++i )
        {
            Quaternion rot(qplist[i].rot);
            Matrix3x4 mat(qplist[i].pos, rot, qplist[i].scale);

            for ( unsigned j = 0; j < numVertices; ++j )
            {
                unsigned char *pOrigDataAlign = (unsigned char *)(origVertBuff.Get() + j * vertexSize);
                unsigned char *pDataAlign = (unsigned char *)(pVertexData + (i * numVertices + j) * vertexSize);
                unsigned sizeRemaining = vertexSize;

                // position
                const Vector3 &vPos = *reinterpret_cast<Vector3*>( pOrigDataAlign );
                Vector3 &nPos = *reinterpret_cast<Vector3*>( pDataAlign );
                nPos = mat * vPos;

                pOrigDataAlign += sizeof(Vector3);
                pDataAlign     += sizeof(Vector3);
                sizeRemaining  -= sizeof(Vector3);

                // for movement
                MoveAccumulator movPt;
                movPt.origPos = nPos;
                movementList_.Push(movPt);

                // bbox
                bbox.Merge(nPos);

                // normal - let's not make any assumptions that the normals exist for every model
                if ( uElementMask & MASK_NORMAL )
                {
                    const Vector3 &vNorm = *reinterpret_cast<Vector3*>( pOrigDataAlign );
                    Vector3 &norm = *reinterpret_cast<Vector3*>( pDataAlign );

                    if ( !overrideNormal )
                    {
                        norm = rot * vNorm;
                    }
                    else
                    {
                        norm = normal;
                    }

                    pOrigDataAlign += sizeof(Vector3);
                    pDataAlign     += sizeof(Vector3);
                    sizeRemaining  -= sizeof(Vector3);
                }
                
                // how about tangents?
                
                // copy everything else excluding what's copied already
                memcpy(pDataAlign, pOrigDataAlign, sizeRemaining);
            }
        }

        //unlock
        pVbuffer->Unlock();
    }

    // replicate indeces
    unsigned newIdxCount = ReplicateIndeces(pIbuffer, numVertices, qplist.Size());

    // set draw range and bounding box
    pGeometry->SetDrawRange(TRIANGLE_LIST, 0, newIdxCount);
    SetBoundingBox( bbox );

    return qplist.Size();
}

unsigned GeomReplicator::ReplicateIndeces(IndexBuffer *idxbuffer, unsigned numVertices, unsigned expandSize)
{
    unsigned numIndeces = idxbuffer->GetIndexCount();
    unsigned origIdxBuffSize = numIndeces * sizeof(unsigned short);
    unsigned newIdxCount = expandSize * numIndeces;
    SharedArrayPtr<unsigned short> origIdxBuff( new unsigned short[numIndeces] );

    void *pIndexData = (void*)idxbuffer->Lock(0, idxbuffer->GetIndexCount());

    // copy orig indeces
    if (pIndexData)
    {
        memcpy(origIdxBuff.Get(), pIndexData, origIdxBuffSize);
        idxbuffer->Unlock();
    }

    // replicate indeces
    if (newIdxCount > 1024*64)
    {
        PODVector<int> newIndexList(newIdxCount);

        for (unsigned i = 0; i < expandSize; ++i)
        {
            for (unsigned j = 0; j < numIndeces; ++j)
            {
                newIndexList[i*numIndeces + j] = i*numVertices + origIdxBuff[j];
            }
        }

        idxbuffer->SetSize(newIdxCount, true);
        idxbuffer->SetData(&newIndexList[0]);
    }
    else
    {
        PODVector<unsigned short> newIndexList(newIdxCount);

        for (unsigned i = 0; i < expandSize; ++i)
        {
            for (unsigned j = 0; j < numIndeces; ++j)
            {
                newIndexList[i*numIndeces + j] = i*numVertices + origIdxBuff[j];
            }
        }

        idxbuffer->SetSize(newIdxCount, false);
        idxbuffer->SetData(&newIndexList[0]);
    }

    return newIdxCount;
}

bool GeomReplicator::ApplyWindVelocity(const PODVector<unsigned> &vertIndecesToMove, unsigned batchCount, 
                                       const Vector3 &velocity, float cycleTimer)
{
    vertIndecesToMove_ = vertIndecesToMove;
    windVelocity_      = velocity;
    cycleTimer_        = cycleTimer;
    batchCount_        = batchCount;
    currentVertexIdx_   = 0;
    timeStepAccum_     = 0.0f;

    // validate vert indeces
    for ( unsigned i = 0; i < vertIndecesToMove.Size(); ++i )
    {
        assert(vertIndecesToMove[i] < numVertsPerGeom && "your vert index must be contained within the original size" );
    }

    timerUpdate_.Reset();
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(GeomReplicator, HandleUpdate));
    return true;
}

void GeomReplicator::MoveVerts()
{
    Geometry *pGeometry = GetModel()->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    unsigned vertexSize = pVbuffer->GetVertexSize();
    unsigned vertsToMove = 0;

    for ( unsigned i = 0; i < batchCount_; ++i )
    {
        unsigned idx = (currentVertexIdx_ + i)*numVertsPerGeom;

        if (idx >= movementList_.Size() )
            break;

        vertsToMove++;

        for ( unsigned j = 0; j < vertIndecesToMove_.Size(); ++j )
        {
            unsigned verIdx = idx + vertIndecesToMove_[j];
            int ielptime = movementList_[verIdx].timer.GetMSec(true);
            if ( ielptime > MaxTime_Elapsed ) ielptime = MaxTime_Elapsed;

            float elapsedTime = (float)ielptime / 1000.0f;
            
            if ( !movementList_[verIdx].reversing )
            {
                Vector3 delta = windVelocity_ * elapsedTime;
                movementList_[verIdx].deltaMovement += delta;
                movementList_[verIdx].timeAccumlated += elapsedTime;

                if ( movementList_[verIdx].timeAccumlated > cycleTimer_ )
                {
                    movementList_[verIdx].reversing = true;
                }
            }
            else
            {
                // slowing it on the way back
                elapsedTime *= 0.5f;
                Vector3 delta = windVelocity_ * elapsedTime;
                movementList_[verIdx].deltaMovement -= delta;
                movementList_[verIdx].timeAccumlated -= elapsedTime;

                if ( movementList_[verIdx].timeAccumlated < 0.0f )
                {
                    movementList_[verIdx].deltaMovement = Vector3::ZERO;
                    movementList_[verIdx].timeAccumlated = 0.0f;
                    movementList_[verIdx].reversing = false;
                }
            }
        }
    }

    // update movement
    unsigned char *pVertexData = (unsigned char*)pVbuffer->Lock(currentVertexIdx_ * numVertsPerGeom, vertsToMove * numVertsPerGeom);

    if ( pVertexData )
    {
        for ( unsigned i = 0; i < batchCount_; ++i )
        {
            unsigned idx = (currentVertexIdx_ + i)*numVertsPerGeom;

            if (idx >= movementList_.Size() )
                break;

            for ( unsigned j = 0; j < vertIndecesToMove_.Size(); ++j )
            {
                unsigned verIdx = idx + vertIndecesToMove_[j];
                unsigned char *pDataAlign = (unsigned char *)(pVertexData + (i*numVertsPerGeom + vertIndecesToMove_[j]) * vertexSize );

                Vector3 &pos = *reinterpret_cast<Vector3*>( pDataAlign );
                pos = movementList_[verIdx].origPos + movementList_[verIdx].deltaMovement;
            }
        }

        pVbuffer->Unlock();
    }

    // update batch idx
    currentVertexIdx_ += batchCount_;

    if (currentVertexIdx_*numVertsPerGeom >= movementList_.Size()) 
    {
        currentVertexIdx_ = 0;
    }
}

void GeomReplicator::StopWindVelocity(bool stop)
{
    if (stop)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(GeomReplicator, HandleUpdate));
    }
    else
    {
        UnsubscribeFromEvent(E_UPDATE);
    }
}

void GeomReplicator::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    float timeStep = eventData[P_TIMESTEP].GetFloat();

    if ( timerUpdate_.GetMSec(false) >= FrameRate_MSec )
    {
        MoveVerts();

        timerUpdate_.Reset();
    }
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
    Vector3 lightDir(0.6f, -1.0f, 0.8f);
    lightNode->SetDirection(lightDir); 
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

        lightDir = -1.0f * lightDir.Normalized();
        vegReplicator_->Replicate(qpList_, lightDir );

        // specify which verts in the geom to move
        // - for the vegbrush model, the top vert indeces are index 2 and 3
        PODVector<unsigned> topVerts;
        topVerts.Push(2);
        topVerts.Push(3);

        // specify the number of geoms to update at a time
        unsigned batchCount = 10000;

        // wind velocity (breeze velocity shown)
        Vector3 windVel(0.2f, -0.2f, 0.2f);

        // specify the cycle timer
        float cycleTimer = 0.4f;

        vegReplicator_->ApplyWindVelocity(topVerts, batchCount, windVel, cycleTimer);
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
    textStatus_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 12);
    textStatus_->SetPosition(10, 10);
    textStatus_->SetColor(Color::WHITE);
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

        stat.AppendWithFormat( "tris: %d fps: %d load time: %d msec", 
                               renderer->GetNumPrimitives(),
                               framesCount_,
                               timeToLoad_);
        //stat += x + y + z;
        textStatus_->SetText(stat);
        framesCount_ = 0;
        fpsTimer_.Reset();
    }

}
