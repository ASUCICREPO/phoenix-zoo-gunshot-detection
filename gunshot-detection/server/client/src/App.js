import React from "react";
import {
  BrowserRouter as Router,
  Routes,
  Route
} from 'react-router-dom';

import DeviceInfo from "./DeviceInfo";
import Main from "./Main";

import NavigationBar from "./NavigationBar";

function App() {
  return (
    <div className="App">
      <NavigationBar />
      <Router>
        <Routes>
          <Route path="/" element={<Main />} />
          <Route path="/device-info" element={<DeviceInfo />} />
        </Routes>
      </Router>
    </div>
  );
}

export default App;
