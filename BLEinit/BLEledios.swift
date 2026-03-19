import SwiftUI
import CoreBluetooth

class BLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var characteristic: CBCharacteristic?
    
    @Published var isConnected = false
    @Published var ledOn = false
    @Published var status = "Starting..."
    
    let serviceUUID = CBUUID(string: "4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    let characteristicUUID = CBUUID(string: "beb5483e-36e1-4688-b7f5-ea07361b26a8")
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func toggleLED() {
        guard let characteristic = characteristic else { return }
        let data = "toggle".data(using: .utf8)!
        peripheral?.writeValue(data, for: characteristic, type: .withResponse)
        ledOn.toggle()
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            status = "Scanning for ESP32..."
            central.scanForPeripherals(withServices: [serviceUUID])
        } else {
            status = "Bluetooth is off"
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        if peripheral.name == "ESP32-LED" {
            status = "Found! Connecting..."
            self.peripheral = peripheral
            self.peripheral?.delegate = self
            central.stopScan()
            central.connect(peripheral)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        isConnected = true
        status = "Connected!"
        peripheral.discoverServices([serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        isConnected = false
        ledOn = false
        status = "Disconnected - reconnecting..."
        central.scanForPeripherals(withServices: [serviceUUID])
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            for service in services {
                if service.uuid == serviceUUID {
                    peripheral.discoverCharacteristics([characteristicUUID], for: service)
                }
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if characteristic.uuid == characteristicUUID {
                    self.characteristic = characteristic
                    status = "Ready!"
                }
            }
        }
    }
}

struct ContentView: View {
    @StateObject private var bleManager = BLEManager()
    
    var body: some View {
        ZStack {
            (bleManager.ledOn ? Color.yellow : Color.gray)
                .opacity(0.3)
                .ignoresSafeArea()
            
            VStack {
                HStack {
                    Circle()
                        .fill(bleManager.isConnected ? Color.green : Color.red)
                        .frame(width: 10, height: 10)
                    Text(bleManager.status)
                        .font(.caption)
                }
                .padding()
                
                Spacer()
                
                Button(action: bleManager.toggleLED) {
                    VStack {
                        Image(systemName: bleManager.ledOn ? "lightbulb.fill" : "lightbulb")
                            .font(.system(size: 100))
                            .foregroundColor(bleManager.ledOn ? .yellow : .gray)
                        Text(bleManager.ledOn ? "ON" : "OFF")
                            .font(.largeTitle)
                    }
                    .frame(width: 250, height: 250)
                    .background(Color.white)
                    .cornerRadius(30)
                    .shadow(radius: 10)
                }
                .disabled(!bleManager.isConnected)
                .opacity(bleManager.isConnected ? 1.0 : 0.5)
                
                Spacer()
            }
        }
    }
}