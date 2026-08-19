
#pragma once

#include <KrakenDemo/KrakenDemoTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace KrakenDemo
{
    class KrakenDemoRequests
    {
    public:
        AZ_RTTI(KrakenDemoRequests, KrakenDemoRequestsTypeId);
        virtual ~KrakenDemoRequests() = default;
        // Put your public methods here
    };

    class KrakenDemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using KrakenDemoRequestBus = AZ::EBus<KrakenDemoRequests, KrakenDemoBusTraits>;
    using KrakenDemoInterface = AZ::Interface<KrakenDemoRequests>;

} // namespace KrakenDemo
