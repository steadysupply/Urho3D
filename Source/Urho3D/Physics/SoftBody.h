#pragma once
#include "../Scene/Component.h"
class btSoftBody;
namespace Urho3D
{
/// Physics soft body component.
class URHO3D_API SoftBody : public Component
{
    URHO3D_OBJECT(SoftBody, Component);
public:
    /// Construct.
    SoftBody(Context* context);
    /// Destruct. Free the soft body and geometries.
    ~SoftBody();
    /// Register object factory.
    static void RegisterObject(Context* context);
    /// Handle logic post-update event where we update the vertex buffer.
void HandlePostUpdate(StringHash eventType, VariantMap& eventData);

/// Remove the soft body.
void ReleaseBody();
/// Create TriMesh from model's geometry.
void CreateTriMesh();
/// Create the soft body from a TriMesh.
bool CreateBodyFromTriMesh(VertexBuffer* vertexBuffer, IndexBuffer* indexBuffer, bool randomizeConstraints = true);
/// Return Bullet soft body.
btSoftBody* GetBody() { return body_; }
/// TODO.
void SetPosition(const Vector3& position);
protected:
    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);
    /// Handle scene being assigned.
    virtual void OnSceneSet(Scene* scene);
    /// Handle node transform being dirtied.
//    virtual void OnMarkedDirty(Node* node);
private:
    /// Create the soft body, or re-add to the physics world with changed flags. Calls UpdateMass().
    void AddBodyToWorld();
    /// Remove the soft body from the physics world.
    void RemoveBodyFromWorld();
    /// Physics world.
WeakPtr<PhysicsWorld> physicsWorld_;
/// Bullet soft body.
btSoftBody* body_;
/// Vertex buffer.
VertexBuffer* vertexBuffer_;
};
}
