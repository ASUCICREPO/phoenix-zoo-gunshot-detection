import React, { useState, useEffect } from "react";
import DatatablePage from "./DatatablePage";
import axios from "axios";

const Main = () => {
  const url = "/api/incidents/";
  const url2 = "/api/devices/";

  const [tableData, setTableData] = useState([]);

  const MINUTE_MS = 60000;

  const columns = [
    {
      Header: "Event Datetime",
      accessor: "eventDate",
    },
    {
      Header: "Device ID",
      accessor: "deviceId",
    },
    {
      Header: "Latitude, Longitude",
      accessor: "coordinates",
    },
    {
      Header: "Google Maps",
      accessor: "link",
      Cell: (e) => (
        <a href={e.value} target="_blank" rel="noreferrer">
          {" "}
          <u>Click here</u>{" "}
        </a>
      ),
    },
  ];

  const getIncidentsAndDevices = async () => {
    let deviceLocations = await axios
      .get(url2, {
        headers: {
          Accept: "application/json",
          "Content-Type": "application/json",
        },
      })
      .then((response) => {
        if (response.status === 200) {
          let data = response.data.devices;
          let rtnObj = {};
          for (let obj of data) {
            let device_id = obj.device_details.device_id;
            let coordinates = obj.device_details.coordinates;
            rtnObj[device_id] = coordinates;
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
          let coordinate_data = "";
          data = data.filter((obj) => obj.device_id in deviceLocations)
          data = data.map((obj) => {
            let eventDate = new Date(Math.trunc(obj.sample_time / 1000) * 1000);
            coordinate_data = deviceLocations[obj.device_id];
            const deviceId = obj.device_data.end_device_ids.device_id;
              return {
                eventDate: `${eventDate.toDateString()}, ${eventDate.toTimeString()}`,
                deviceId: deviceId,
                coordinates: coordinate_data,
                link: `https://www.google.com/maps/search/?api=1&query=${coordinate_data}`,
              };
          });
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

export default Main;
