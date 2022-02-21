import React, { useState, useEffect } from "react";
import DatatablePage from "./DatatablePage";
import axios from "axios";

const DeviceInfo = () => {
    const url = "/api/incidents/";
    const url2 = "/api/devices/";

    const [tableData, setTableData] = useState([]);

    const MINUTE_MS = 60000;

    const columns = [
        {
            Header: "Device ID",
            accessor: "deviceId",
        },
        {
            Header: "Battery Voltage",
            accessor: "batteryVoltage",
        },
        {
            Header: "SNR",
            accessor: "snr",
        },
        {
            Header: "RSSI",
            accessor: "rssi",
        },
        {
            Header: "Time",
            accessor: "time",
        },
    ];


    const getIncidentsAndDevices = async () => {
        let deviceList = await axios
            .get(url2, {
                headers: {
                    Accept: "application/json",
                    "Content-Type": "application/json",
                },
            })
            .then((response) => {
                if (response.status === 200) {
                    let data = response.data.devices;
                    let rtnObj = [];
                    for (let obj of data) {
                        let device_id = obj.device_details.device_id;
                        rtnObj.push(device_id);
                    }
                    return rtnObj;
                }
            });
        await axios
            .get(url, {
                headers: {
                    Accept: "application/json",
                    "Content-Type": "application/json",
                },
            })
            .then((response) => {
                if (response.status === 200) {
                    let data = response.data.incidents;
                    data = data.filter((obj) => deviceList.includes(obj.device_id));
                    data = data.map((obj) => {
                        const snr = obj.device_data.uplink_message.rx_metadata[0].snr;
                        const rssi = obj.device_data.uplink_message.rx_metadata[0].rssi;
                        const deviceId = obj.device_data.end_device_ids.device_id;
                        const time = obj.device_data.received_at;
                        let voltage = ''
                        if (obj.device_data.uplink_message.frm_payload) {
                            voltage = Buffer.from(obj.device_data.uplink_message.frm_payload, 'base64').toString('ascii');
                        } else {
                            voltage = 'undefined'
                        }
                        return {
                            batteryVoltage: voltage,
                            deviceId: deviceId,
                            snr: snr,
                            rssi: rssi,
                            time: time
                        };
                    });

                    data = data.filter((obj) => obj.batteryVoltage.match(/^-?\d+\.?\d*$/) != null)

                    setTableData(data);
                }
            });
    };

    useEffect(() => {
        getIncidentsAndDevices();
        const interval = setInterval(() => {
            getIncidentsAndDevices();
        }, MINUTE_MS);

        return () => clearInterval(interval);
    }, []);
    return (
        <div>
            <DatatablePage columns={columns} data={tableData} />
        </div>
    );
};

export default DeviceInfo;
