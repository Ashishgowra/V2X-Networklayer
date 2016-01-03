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

import inet.status.NodeStatus;
import inet.base.NotificationBoard;
import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;
import org.car2x.veins.inet.StandardHost;
import inet.networklayer.autorouting.ipv4.HostAutoConfigurator;
import inet.networklayer.IManetRouting;
import org.car2x.veins.modules.routing.GPSR;
import inet.networklayer.ipv4.RoutingTable;
import inet.nodes.inet.NetworkLayer;
import inet.networklayer.common.InterfaceTable;
import inet.transport.IUDP;
import inet.applications.IUDPApp;
import inet.networklayer.autorouting.ipv4.IPv4NodeConfigurator;

module RSU
{
    parameters:
        @node;
        string applType; //type of the application layer
        string nicType = default("Nic80211p"); // type of network interface card
        string routingProtocol @enum("AODVUU","DYMOUM","DYMO","DSRUU","OLSR","OLSR_ETX","DSDV_2","Batman") = default("");
        bool IPForward = default(true);
        bool useGPSR = default(false);
        int numUdpApps = default(0);
        string udpType = default(firstAvailable("UDP","UDP_None"));
        bool forwardMulticast = default(false);
        string routingFile = default("");
        bool hasStatus = default(false);
        @display("bgb=700,500");
    gates:
        input veinsradioIn @directIn; // gate for sendDirect
    submodules:
        status: NodeStatus if hasStatus {
            @display("p=81,72");
        }

        notificationBoard: NotificationBoard {
            parameters:
                @display("p=81,130");
        }

        interfaceTable: InterfaceTable {
            parameters:
                @display("p=73,311;is=s");
        }

        routingTable: RoutingTable {
            parameters:
                @display("p=73,262;is=s");
                IPForward = IPForward;
                forwardMulticast = forwardMulticast;
                routingFile = routingFile;
        }

        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=329,388");
        }
        mobility: BaseMobility {
            parameters:
                @display("p=81,204;i=block/cogwheel");
        }


        manetrouting: <routingProtocol> like IManetRouting if routingProtocol != "" {
            @display("p=516,311");
        }

        networkLayer: NetworkLayer {
            parameters:
                @display("p=329,262;q=queue");
        }

        gpsr: GPSR if useGPSR {
            @display("i=block/routing;p=516,225");
        }

        autoConfig: HostAutoConfigurator {
            @display("p=73,357");
        }

        udpApp[numUdpApps]: <> like IUDPApp {
            parameters:
                @display("p=329,54,row,60");
        }
        udp: <udpType> like inet.transport.IUDP {
            parameters:
                @display("p=329,141");
        }

    connections allowunconnected:

        veinsradioIn --> nic.radioIn;
        nic.ipOut --> networkLayer.ifIn++;
        nic.ipIn <-- networkLayer.ifOut++;

        networkLayer.transportOut++ --> manetrouting.from_ip if routingProtocol != "";
        networkLayer.transportIn++ <-- manetrouting.to_ip if routingProtocol != "";

        if (useGPSR) {
            networkLayer.transportOut++ --> gpsr.ipIn;
            networkLayer.transportIn++ <-- gpsr.ipOut;
        }

        for i=0..numUdpApps-1 {
            udpApp[i].udpOut --> udp.appIn++;
            udpApp[i].udpIn <-- udp.appOut++;
        }

        udp.ipOut --> networkLayer.transportIn++;
        udp.ipIn <-- networkLayer.transportOut++;

}
