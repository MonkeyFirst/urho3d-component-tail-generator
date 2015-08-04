# urho3d-component-tail-generator
Tail generator component for Urho3d engine

Using:

```
#include "TailGenerator.h"

void Game::Start()
{
    ...
    TailGenerator::RegisterObject(context_);
    ...
}

void Game::CreateScene()
{
    Node *tailNode = modelNode->CreateChild();
    tailNode->Translate(Vector3(0.0f, 1.0f, 0.1f), TransformSpace::TS_LOCAL); // translate a tail relatively a object
    TailGenerator* tailGen = tailNode->CreateComponent<TailGenerator>();
    tailGen->SetTailLength(0.1f); // set segment length
    tailGen->SetNumTails(50);     // set num of segments
    tailGen->SetWidthScale(4.0f); // side scale
    tailGen->SetColorForHead(Color(1.0f, 1.0f, 1.0f));
    tailGen->SetColorForTip(Color(0.0f, 0.0f, 1.0f));
}
```