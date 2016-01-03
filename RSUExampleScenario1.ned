//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;
import inet.networklayer.autorouting.ipv4.IPv4NetworkConfigurator;
import inet.world.radio.ChannelControl;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.base.LifecycleController;

network RSUExampleScenario1 extends Scenario
{
    parameters:
        @display("bgb=8000,8000;bgl=10");
        int numRsu = default(1);
    submodules:
//        rsu[numRsu]: RSU {
//            @display("p=7500,7500;b=10,10,oval");
//        }

        configurator: IPv4NetworkConfigurator {
            config = xml("<config><interface hosts='*' address='192.168.x.x' netmask='255.255.0.0'/></config>");
            @display("p=73,159");
        }

        routingTableRecorder: RoutingTableRecorder {
            parameters:
                @display("p=73,218");
        }
        lifecycleController: LifecycleController {
            parameters:
                @display("p=73,277");
        }

}
