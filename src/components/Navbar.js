import React from "react";

const handleReport = () => {
  console.log("포트홀 신고 버튼 클릭");
};

const Navbar = () => {
  return (
    <nav className="bg-white shadow-sm">
      <div className="max-w-8xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between h-16">
          <div className="flex">
            <div className="flex-shrink-0 flex items-center">
              <img
                className="h-8 w-auto"
                src="https://img.freepik.com/premium-psd/quadcopter-drone-png-isolated-transparent-background_753500-620.jpg"
                alt="로고"
              />
            </div>
          </div>
          <div className="flex items-center">
            <button
              onClick={handleReport}
              className="rounded bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 text-sm font-medium m-6"
            >
              <i className="fas fa-plus mr-2"></i>포트홀 신고
            </button>
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;
