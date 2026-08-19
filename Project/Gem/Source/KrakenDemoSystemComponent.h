
#pragma once

#include <AzCore/Component/Component.h>

#include <KrakenDemo/KrakenDemoBus.h>

namespace KrakenDemo
{
    class KrakenDemoSystemComponent
        : public AZ::Component
        , protected KrakenDemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(KrakenDemoSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        KrakenDemoSystemComponent();
        ~KrakenDemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // KrakenDemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
