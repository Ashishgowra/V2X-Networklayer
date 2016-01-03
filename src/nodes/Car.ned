//
// Copyright (C) 2008 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package org.car2x.veins.nodes;

import org.car2x.veins.base.modules.*;
import org.car2x.veins.inet.StandardHost;
import inet.networklayer.autorouting.ipv4.HostAutoConfigurator;
import inet.networklayer.IManetRouting;
import org.car2x.veins.modules.routing.GPSR;


module Car extends StandardHost
{
    parameters:
        @node;
        networkLayer.configurator.networkConfiguratorModule = "";
        string routingProtocol @enum("AODVUU","DYMOUM","DYMO","DSRUU","OLSR","OLSR_ETX","DSDV_2","Batman") = default("");
        IPForward = default(true);
        bool useGPSR = default(false);
        string applType = default("");
        
        
    submodules:
        manetrouting: <routingProtocol> like IManetRouting if routingProtocol != "" {
            @display("p=522,307");
        }
        autoConfig: HostAutoConfigurator {
            @display("p=171,335");
        }
        gpsr: GPSR if useGPSR {
            @display("i=block/routing;p=527,248");
        }
//        appl: <applType> like org.car2x.veins.base.modules.IBaseApplLayer {
//            parameters:
//                @display("p=147,225");
//        }

    connections allowunconnected:

        networkLayer.transportOut++ --> manetrouting.from_ip if routingProtocol != "";
        networkLayer.transportIn++ <-- manetrouting.to_ip if routingProtocol != "";

        if (useGPSR) {
            networkLayer.transportOut++ --> gpsr.ipIn;
            networkLayer.transportIn++ <-- gpsr.ipOut;
        }

//        nic.wsmOut --> appl.lowerLayerIn;
//        nic.wsmIn <-- appl.lowerLayerOut;
//        nic.upperControlOut --> appl.lowerControlIn;
//        nic.upperControlIn <-- appl.lowerControlOut;
}
