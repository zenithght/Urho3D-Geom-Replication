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

#pragma once

#include "Sample.h"

namespace Urho3D
{
class Node;
class Scene;
}

//=============================================================================
//=============================================================================
struct PRotScale
{
    Vector3     pos;
    Quaternion  rot;
    float       scale;
};

class GeomReplicator : public StaticModel
{
    URHO3D_OBJECT(GeomReplicator, StaticModel);
public:
    static void RegisterObject(Context* context)
    {
        context->RegisterFactory<GeomReplicator>();
    }

    GeomReplicator(Context *context) : StaticModel(context)
    {
    }

    virtual ~GeomReplicator()
    {
    }

    unsigned Replicate(const PODVector<PRotScale> &qplist);

protected:
    unsigned ReplicateIndeces(IndexBuffer *idxbuffer, unsigned numVertices, unsigned expandSize);
};

//=============================================================================
//=============================================================================
class StaticScene : public Sample
{
    URHO3D_OBJECT(StaticScene, Sample);

public:
    StaticScene(Context* context);

    virtual void Setup();
    virtual void Start();

protected:
    void CreateScene();
    void CreateStatusText();
    void SetupViewport();
    void MoveCamera(float timeStep);
    void SubscribeToEvents();
    void HandleUpdate(StringHash eventType, VariantMap& eventData);

protected:
    WeakPtr<Text> textStatus_;

    Timer         fpsTimer_;
    int           framesCount_;

    PODVector<PRotScale> qpList_;

    unsigned      timeToLoad_;
    Timer         keyDebounceTimer_;

    // replicator
    SharedPtr<GeomReplicator> vegReplicator_;
    WeakPtr<Node> nodeRep_;
};
