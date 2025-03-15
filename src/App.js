import React, { useState } from "react";
import Navbar from "./components/Navbar";
import Sidebar from "./components/Sidebar";
import MapComponent from "./components/MapComponent";

function App() {
  const [coords, setCoords] = useState(null);

  return (
    <div className="min-h-screen flex flex-col">
      <Navbar />
      <div className="flex-1 flex">
        {/* Sidebar에 onCoordsChange prop 전달 */}
        <Sidebar onCoordsChange={setCoords} />
        <div className="flex-1 relative">
          {/* MapComponent에 현재 좌표 props 전달 */}
          <MapComponent coords={coords} />
          {/* 지도 상단 좌측 컨트롤 버튼 */}
          <div className="absolute left-4 top-4 bg-white rounded-lg shadow">
            <button className="!rounded-button p-2 hover:bg-gray-100">
              <i className="fas fa-plus text-gray-600"></i>
            </button>
            <button className="!rounded-button p-2 hover:bg-gray-100 border-t border-gray-200">
              <i className="fas fa-minus text-gray-600"></i>
            </button>
            <button className="!rounded-button p-2 hover:bg-gray-100 border-t border-gray-200">
              <i className="fas fa-location-crosshairs text-gray-600"></i>
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;
  