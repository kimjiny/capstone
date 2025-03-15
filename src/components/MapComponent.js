import React, { useEffect, useRef } from "react";

const MapComponent = ({ coords }) => {
  const mapRef = useRef(null);
  const mapInstance = useRef(null);

  useEffect(() => {
    const { kakao } = window;
    if (!kakao || !mapRef.current) return;

    const mapContainer = mapRef.current;
    const mapOption = {
      center: new kakao.maps.LatLng(37.29803324573562, 126.83879424585795),
      level: 7,
    };
    const map = new kakao.maps.Map(mapContainer, mapOption);
    // map 인스턴스를 ref에 저장합니다.
    mapInstance.current = map;

    // 기본 마커들 (예시)
    const positions = [
      {
        title: "처리중",
        latlng: new kakao.maps.LatLng(37.498095, 127.02761),
      },
      {
        title: "미완료",
        latlng: new kakao.maps.LatLng(37.493923, 127.014656),
      },
    ];

    positions.forEach((position) => {
      new kakao.maps.Marker({
        map,
        position: position.latlng,
        title: position.title,
      });
    });

    // 백엔드에서 포트홀 데이터를 가져와 마커 생성
    fetch("http://localhost:3000/potholes")
      .then((response) => response.json())
      .then((data) => {
        data.forEach((pothole) => {
          const marker = new kakao.maps.Marker({
            map,
            position: new kakao.maps.LatLng(pothole.latitude, pothole.longitude),
            title: pothole.location,
          });

          const infowindow = new kakao.maps.InfoWindow({
            content: `<div style="padding:5px;">위치: ${pothole.location}<br>상태: ${pothole.status}</div>`,
          });

          kakao.maps.event.addListener(marker, "click", () => {
            infowindow.open(map, marker);
          });
        });
      })
      .catch((error) => console.error("포트홀 정보 가져오기 실패:", error));
  }, []);

  // 좌표 값이 변경될 때마다 지도 중심 업데이트
  useEffect(() => {
    if (coords && mapInstance.current) {
      const { kakao } = window;
      mapInstance.current.setCenter(new kakao.maps.LatLng(coords.lat, coords.lng));
    }
  }, [coords]);

  return (
    <div
      ref={mapRef}
      style={{ width: "100%", height: "100%" }}
      className="absolute inset-0"
    />
  );
};

export default MapComponent;
