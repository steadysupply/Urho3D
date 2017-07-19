#include "../Precompiled.h"
#include "../Core/Context.h"
#include "../Core/CoreEvents.h"
#include "../Graphics/Geometry.h"
#include "../Graphics/IndexBuffer.h"
#include "../IO/Log.h"
#include "../Graphics/Material.h"
#include "../Graphics/Model.h"
#include "../Physics/PhysicsUtils.h"
#include "../Physics/PhysicsWorld.h"
#include "../Resource/ResourceCache.h"
#include "../Scene/Scene.h"
#include "../Scene/SceneEvents.h"
#include "../Physics/SoftBody.h"
#include "../Graphics/StaticModel.h"
#include "../Graphics/VertexBuffer.h"
#include <Bullet/BulletSoftBody/btSoftBody.h>
#include <Bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <Bullet/BulletSoftBody/btSoftBodyHelpers.h>
namespace Urho3D
{
extern const char* PHYSICS_CATEGORY;
SoftBody::SoftBody(Context* context) :
    Component(context),
    body_(NULL),
    vertexBuffer_(NULL)
{
}
SoftBody::~SoftBody()
{
    if (body_)
    {
        delete body_;
        body_ = NULL;
    }
    // We don't own the vertsbuffer
vertexBuffer_ = NULL;
}
void SoftBody::RegisterObject(Context* context)
{
    context->RegisterFactory<SoftBody>(PHYSICS_CATEGORY);
}
void SoftBody::OnNodeSet(Node* node)
{
    if (node)
        node->AddListener(this);
}
void SoftBody::OnSceneSet(Scene* scene)
{
    if (scene)
    {
        if (scene == node_)
            URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");
        physicsWorld_ = scene->GetOrCreateComponent<PhysicsWorld>();
    physicsWorld_->AddSoftBody(this);

    AddBodyToWorld();
}
else
{
    ReleaseBody();

    if (physicsWorld_)
        physicsWorld_->RemoveSoftBody(this);
}
}
void SoftBody::AddBodyToWorld()
{
    if (!physicsWorld_)
        return;
    if (body_)
{
    btSoftRigidDynamicsWorld* world = (btSoftRigidDynamicsWorld*)physicsWorld_->GetWorld();
    world->addSoftBody(body_);
}
}
void SoftBody::ReleaseBody()
{
    if (body_)
    {
        RemoveBodyFromWorld();
        delete body_;
        body_ = NULL;
    }
}
void SoftBody::RemoveBodyFromWorld()
{
    if (body_)
    {
        if (physicsWorld_)
        {
            btSoftRigidDynamicsWorld* pSoftRigidWorld = (btSoftRigidDynamicsWorld *)physicsWorld_->GetWorld();
            pSoftRigidWorld->removeSoftBody(body_);
        }
    }
}
void SoftBody::CreateTriMesh()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Scene* scene = GetScene();
    // Get model
    StaticModel* model = node_->GetComponent<StaticModel>();
    if (!model)
        return;
    Model* originalModel = model->GetModel();
    if (!originalModel)
        return;

    // Clone model
    SharedPtr<Model> cloneModel = originalModel->Clone();
    model->SetModel(cloneModel);

    // Get the vertex and index buffers from the first geometry's first LOD level
    VertexBuffer* vertexBuffer = cloneModel->GetGeometry(0, 0)->GetVertexBuffer(0);
    IndexBuffer* indexBuffer = cloneModel->GetGeometry(0, 0)->GetIndexBuffer();

    // Cretae soft body from TriMesh
    CreateBodyFromTriMesh(vertexBuffer, indexBuffer);
}
bool SoftBody::CreateBodyFromTriMesh(VertexBuffer* vertexBuffer, IndexBuffer* indexBuffer, bool randomizeConstraints)
{
    bool bConstructed = false;
    if (vertexBuffer && indexBuffer)
{
    btAlignedObjectArray<bool> chks;
    btAlignedObjectArray<btVector3> vtx;

    // Save vertexbuffer ptr
    vertexBuffer_ = vertexBuffer;

    // Copy vertex buffer
    const unsigned char* pVertexData = (const unsigned char*)vertexBuffer_->Lock(0, vertexBuffer_->GetVertexCount());

    if (pVertexData)
    {
        unsigned numVertices = vertexBuffer_->GetVertexCount();
        unsigned vertexSize = vertexBuffer_->GetVertexSize();

        vtx.resize(numVertices);

        // Copy the original verts
        for (unsigned i = 0; i < numVertices; ++i)
        {
            const Vector3& src = *reinterpret_cast<const Vector3*>(pVertexData + i * vertexSize);
            vtx[i] = ToBtVector3(src);
        }
        vertexBuffer_->Unlock();
    }

    // Create softbody
    physicsWorld_ = GetScene()->GetComponent<PhysicsWorld>();
    body_ = new btSoftBody(physicsWorld_->GetSoftBodyInfo(), vtx.size(), &vtx[0], 0);

    // Copy indexbuffer
    const unsigned* pIndexData = (const unsigned*)indexBuffer->Lock(0, indexBuffer->GetIndexCount());
    const unsigned short* pUShortData = (const unsigned short*)pIndexData;
    if (pIndexData)
    {
        unsigned numIndices = indexBuffer->GetIndexCount();
        unsigned indexSize = indexBuffer->GetIndexSize();

        int ntriangles = (int)numIndices / 3;

        int maxidx = 0;
        int i; //,j,ni;

        if (indexSize == sizeof(unsigned short))
        {
            for (i = 0; i < (int)numIndices; ++i)
            {
                unsigned uidx = pUShortData[i];
                maxidx = Max(uidx, maxidx);
            }
        }
        else if (indexSize == sizeof(unsigned))
        {
            for (i = 0; i < (int)numIndices; ++i)
            {
                unsigned uidx = pIndexData[i];
                maxidx = Max(uidx, maxidx);
            }
        }
        ++maxidx;
        chks.resize(maxidx * maxidx, false);

        for (i = 0; i < (int)numIndices; i += 3)
        {
            int idx[3];
            if (indexSize == sizeof(unsigned short))
            {
                idx[0] = (int)pUShortData[i];
                idx[1] = (int)pUShortData[i + 1];
                idx[2] = (int)pUShortData[i + 2];
            }
            else
            {
                idx[0] = (int)pIndexData[i];
                idx[1] = (int)pIndexData[i + 1];
                idx[2] = (int)pIndexData[i + 2];
            }

            #define IDX(_x_, _y_) ((_y_) * maxidx + (_x_))
            for (int j=2, k=0; k<3; j = k++)
            {
                if (!chks[IDX(idx[j], idx[k])])
                {
                    chks[IDX(idx[j], idx[k])] = true;
                    chks[IDX(idx[k], idx[j])] = true;
                    body_->appendLink(idx[j], idx[k]);
                }
            }
            #undef IDX
            body_->appendFace(idx[0], idx[1], idx[2]);
        }
        indexBuffer->Unlock();
    }

    if (randomizeConstraints)
        body_->randomizeConstraints();

    // Straight out of Bullet's softbody demo for trimesh
    body_->m_materials[0]->m_kLST = 0.1;
    body_->m_cfg.kMT = 0.05;

    btMatrix3x3 m;
    m.setEulerZYX(0, 0, 0);

    // Create methods for these
    body_->transform(btTransform(m, btVector3(0, 4, 0)));
    body_->scale(btVector3(2, 2, 2));
    body_->setTotalMass(50, true);
    body_->setPose(true, true);

    bConstructed = true;
}

AddBodyToWorld();
SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(SoftBody, HandlePostUpdate));

return bConstructed;
}
void SoftBody::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    // Update vertex buffer
    if (body_ && vertexBuffer_)
    {
        unsigned char* pVertexData = (unsigned char*)vertexBuffer_->Lock(0, vertexBuffer_->GetVertexCount());
        // Copy soft body vertices back into the model vertex buffer
    if (pVertexData)
    {
        unsigned numVertices = vertexBuffer_->GetVertexCount();
        unsigned vertexSize = vertexBuffer_->GetVertexSize();

        // Copy original vertex positions
        for (unsigned i = 0; i < body_->m_nodes.size(); ++i)
        {
            btSoftBody::Node& n = body_->m_nodes[i];
            Vector3& src = *reinterpret_cast<Vector3*>(pVertexData + i * vertexSize);
            src = ToVector3(n.m_x);
        }
        vertexBuffer_->Unlock();
    }
}
}
void SoftBody::SetPosition(const Vector3& position)
{
    if (body_)
    {
        body_->transform(btTransform(btQuaternion::getIdentity(), ToBtVector3(position)));
        MarkNetworkUpdate();
    }
}
}
