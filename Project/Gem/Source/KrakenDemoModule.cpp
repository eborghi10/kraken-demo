
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "KrakenDemoSystemComponent.h"

#include <KrakenDemo/KrakenDemoTypeIds.h>

namespace KrakenDemo
{
    class KrakenDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(KrakenDemoModule, KrakenDemoModuleTypeId, AZ::Module);
        AZ_CLASS_ALLOCATOR(KrakenDemoModule, AZ::SystemAllocator);

        KrakenDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                KrakenDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<KrakenDemoSystemComponent>(),
            };
        }
    };
}// namespace KrakenDemo

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), KrakenDemo::KrakenDemoModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_KrakenDemo, KrakenDemo::KrakenDemoModule)
#endif
