
#include <AzCore/Serialization/SerializeContext.h>

#include "KrakenDemoSystemComponent.h"

#include <KrakenDemo/KrakenDemoTypeIds.h>

namespace KrakenDemo
{
    AZ_COMPONENT_IMPL(KrakenDemoSystemComponent, "KrakenDemoSystemComponent",
        KrakenDemoSystemComponentTypeId);

    void KrakenDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<KrakenDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void KrakenDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("KrakenDemoService"));
    }

    void KrakenDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("KrakenDemoService"));
    }

    void KrakenDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void KrakenDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    KrakenDemoSystemComponent::KrakenDemoSystemComponent()
    {
        if (KrakenDemoInterface::Get() == nullptr)
        {
            KrakenDemoInterface::Register(this);
        }
    }

    KrakenDemoSystemComponent::~KrakenDemoSystemComponent()
    {
        if (KrakenDemoInterface::Get() == this)
        {
            KrakenDemoInterface::Unregister(this);
        }
    }

    void KrakenDemoSystemComponent::Init()
    {
    }

    void KrakenDemoSystemComponent::Activate()
    {
        KrakenDemoRequestBus::Handler::BusConnect();
    }

    void KrakenDemoSystemComponent::Deactivate()
    {
        KrakenDemoRequestBus::Handler::BusDisconnect();
    }
}
