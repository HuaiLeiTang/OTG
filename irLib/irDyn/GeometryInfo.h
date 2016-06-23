/*!
*	\file	GeometryInfo.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	GeometryInfo class
*/

#pragma once

#include <string>
#include <list>
#include <memory>

#include <irMath\Constant.h>
#include <irMath\LieGroup.h>

namespace irLib
{
	namespace irDyn
	{
		class GeometryInfo;

		typedef std::shared_ptr< GeometryInfo > GeometryInfoPtr;

		class GeometryInfo
		{
		public:
			/// Geometry type의 종류
			enum GEOMETRY_TYPE
			{
				_BOX,		///< Box 상자
				_SPHERE,	///< Sphere 구
				_CAPSULE,	///< Capsule 캡슐
				_CYLINDER,	///< Cylinder 원통
				_MESH,		///< Mesh mesh의 정보를 저장
				_USERMODEL	///< UserModel 사용자가 직접 만드는 모델
			};

			/// 생성자
			GeometryInfo(const GEOMETRY_TYPE& Type, ///< Type
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : _Type(Type), _T(T), _Color(Color) {}

			/// 소멸자, 가상 클래스로 만들어 줍니다.
			virtual ~GeometryInfo() = 0;

			/// Type을 설정합니다.
			void setType(const GEOMETRY_TYPE& Type)
			{
				_Type = Type;
			}
			/// Frame의 위치를 설정합니다.
			void setFrame(const irMath::SE3& T)
			{
				_T = T;
			}
			/// (R, G, B, Alpha) 색을 설정합니다.
			void setColor(const irMath::Real& R, ///< Red
				const irMath::Real& G, ///< Green
				const irMath::Real& B, ///< Blue
				const irMath::Real& Alpha = (1.0) /// Alpha
				)
			{
				_Color << R, G, B, Alpha;
			}
			/// (R, G, B, Alpha) 색을 설정합니다.
			void setColor(const irMath::Vector4& Color // 색 [R; G; B; Alpha]
				)
			{
				_Color = Color;
			}

			/// 현재 geometry의 type을 가지고 옵니다.
			const GEOMETRY_TYPE& getType() const
			{
				return _Type;
			}
			/**
			*	\return irMath::Vector4 [R; G; B; Alpha]
			*	\brief 현재 설정 되어있는 색을 알려줍니다.
			*/
			const irMath::Vector4& getColor() const
			{
				return _Color;
			}
			/**
			*	\return irMath::SE3 Frame
			*	\brief 현재 설정 되어 있는 Frame을 알려줍니다.
			*/
			const irMath::SE3& getTransform() const
			{
				return _T;
			}

			/// Geometry 깊은 복사, 자식 클래스가 생길경우 copy함수 수정이 필요합니다.
			virtual GeometryInfoPtr copy() const = 0;

		private:
			GEOMETRY_TYPE _Type; ///< Geometry type을 저장하는 변수
			irMath::SE3 _T; ///< Frame의 위치를 저장하고 있는 변수
			irMath::Vector4 _Color; ///< R, G, B, Alpha 색을 저장하는 변수

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		/**
		*	\class Box
		*	\brief Geometry box의 정보를 저장하는 클래스, GEOMETRY_TYPE = _BOX
		*/
		class Box :public GeometryInfo
		{
		public:
			/// 기본 생성자, 가로=0, 세로=0, 높이=0로 초기화됩니다.
			Box(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(0.0), _height(0.0), _depth(0.0) {}
			/// 입력으로 들어온 가로, 세로, 높이로 초기화됩니다.
			Box(const irMath::Real& width, ///< 가로
				const irMath::Real& depth, ///< 세로
				const irMath::Real& height, ///< 높이
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(width), _height(height), _depth(depth) {}
			/// 입력으로 들어온 config를 이용하여 초기화를 합니다.
			Box(const irMath::Vector3& config, ///< [가로; 세로; 높이]
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(config(0)), _height(config(1)), _depth(config(2)) {}

			/// 소멸자
			~Box() {}

			/// 입력으로 들어온 가로, 세로, 높이로 값을 바꿉니다.
			void setDimension(const irMath::Real& width, ///< 가로
				const irMath::Real& depth, ///< 세로
				const irMath::Real& height ///< 높이
				);
			/// 입력으로 들어온 config로 가로, 세로, 높이를 바꿔줍니다.
			void setDimension(const irMath::Vector3& config ///< [가로; 세로; 높이]
				);
			/// 입력으로 들어온 값을 한변의 길이로 하는 정육면체를 만듭니다.
			void setCube(const irMath::Real& length ///< 정육면체의 한변의 길이
				);

			/**
			*	\return irMath::Vector3 [가로; 세로; 높이]
			*	\brief 현재 설정 되어있는 가로, 세로, 높이를 알려줍니다.
			*/
			irMath::Vector3 getDimension() const;

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _width, _depth, _height;
		};

		/**
		*	\class Sphere
		*	\brief Geometry sphere의 정보를 저장하는 클래스, GEOMETRY_TYPE = _SPHERE
		*/
		class Sphere :public GeometryInfo
		{
		public:
			/// 기본 생성자, 반지름=0로 초기화됩니다.
			Sphere(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(0.0) {}
			/// 입력으로 들어온 반지름으로 초기화됩니다.
			Sphere(const irMath::Real& radius, ///< 반지름
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(radius) {}

			/// 소멸자
			~Sphere() {}

			/// 입력으로 들어온 radius로 반지름 값을 바꿉니다.
			void setRadius(const irMath::Real& radius ///< 반지름
				);

			/**
			*	\return 반지름
			*	\brief 현재 설정 되어있는 반지름 값을 알려줍니다.
			*/
			const irMath::Real& getRadius() const;

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius;
		};

		/**
		*	\class Capsule
		*	\brief Geometry capsule의 정보를 저장하는 클래스, GEOMETRY_TYPE = _CAPSULE
		*/
		class Capsule :public GeometryInfo
		{
		public:
			/// 기본 생성자, 반지름=0, 높이=0로 초기화됩니다.
			Capsule(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(0.0), _height(0.0) {}
			/// 입력으로 들어온 반지름과 높이로 초기화됩니다.
			Capsule(const irMath::Real& radius, ///< 반지름
				const irMath::Real& height, ///< 높이
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(radius), _height(height) {}

			/// 소멸자
			~Capsule() {}

			/// 입력으로 들어온 반지름, 높이로 값을 바꿉니다.
			void setDimension(const irMath::Real& radius, ///< 가로
				const irMath::Real& height ///< 높이
				);
			/// 입력으로 들어온 config로 반지름, 높이를 바꿔줍니다.
			void setDimension(const irMath::Vector2& config ///< [반지름: 높이]
				);

			/**
			*	\return irMath::Vector2 [반지름; 높이]
			*	\brief 현재 설정 되어있는 반지름, 높이를 알려줍니다.
			*/
			irMath::Vector2 getDimension() const;

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius, _height;
		};

		/**
		*	\class Cylinder
		*	\brief Geometry cylinder의 정보를 저장하는 클래스, GEOMETRY_TYPE = _CYLINDER
		*/
		class Cylinder :public GeometryInfo
		{
		public:
			/// 기본 생성자, 반지름=0, 높이=0로 초기화됩니다.
			Cylinder(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(0.0), _height(0.0) {}
			/// 입력으로 들어온 반지름과 높이로 초기화됩니다.
			Cylinder(const irMath::Real& radius, ///< 반지름
				const irMath::Real& height, ///< 높이
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(radius), _height(height) {}

			/// 소멸자
			~Cylinder() {}

			/// 입력으로 들어온 반지름, 높이로 값을 바꿉니다.
			void setDimension(const irMath::Real& radius, ///< 가로
				const irMath::Real& height ///< 높이
				);
			/// 입력으로 들어온 config로 반지름, 높이를 바꿔줍니다.
			void setDimension(const irMath::Vector2& config ///< [반지름: 높이]
				);

			/**
			*	\return irMath::Vector2 [반지름; 높이]
			*	\brief 현재 설정 되어있는 반지름, 높이를 알려줍니다.
			*/
			irMath::Vector2 getDimension() const;

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius, _height;
		};

		/**
		*	\class Mesh
		*	\brief Geometry mesh의 정보를 저장하는 클래스, GEOMETRY_TYPE = _MESH
		*/
		class Mesh :public GeometryInfo
		{
		public:
			/// 기본 생성자, 주소는 NULL로 초기화 됩니다,
			Mesh(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url("") {}
			/// Mesh 파일이 저장되어 있는 주소 값을 이용하여 초기화합니다.
			Mesh(const std::string& url, ///< Mesh 파일이 저장되어 있는 주소
				const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url(url) {}

			/// Mesh 파일이 저장되어 있는 주소 값을 받아 저장합니다.
			void setUrl(const std::string& url ///< Mesh 파일이 저장되어 있는 주소
				)
			{
				_url = url;
			}

			/// 현재 설정 되어있는 주소 값을 알려줍니다.
			const std::string& getUrl() const
			{
				return _url;
			}

			void		setDimension(irMath::Real scale) { _scale = scale; }
			irMath::Real	getDimension() const { return _scale; }

			/**
			*	\return 존재하면 True, 존재하지 않으면 False
			*	\brief 현재 설정 되어있는 주소에 파일이 존재하는지 판단
			*/
			bool isValid() const;

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			std::string _url;
			irMath::Real	_scale = 1;
		};

		/**
		*	\class UserModel
		*	\brief Geometry 정보를 사용자가 마음대로 만들 수 있는 클래스, GEOMETRY_TYPE = _USERMODEL
		*	\todo addList 무한루프 돌게 하지 않게 하기, 검색, 삭제 함수 만들기
		*/
		class UserModel :public GeometryInfo
		{
		public:
			/// 생성자
			UserModel(const irMath::SE3& T = (irMath::SE3()), ///< Frame위치
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) 색
				) : GeometryInfo(GeometryInfo::_USERMODEL, T, Color), _GeometryList() {}

			/// srGeometry주소와 irMath::SE3값을 받아서 저장합니다.
			void addList(const GeometryInfoPtr& shape, const irMath::SE3& T);

			/// List로부터 값을 찾거나 설정을 바꾸고 싶을 때 사용합니다.
			std::list< std::pair< GeometryInfoPtr, irMath::SE3 > >& getList()
			{
				return _GeometryList;
			}

			// 깊은 복사
			GeometryInfoPtr copy() const;

		private:
			std::list< std::pair< GeometryInfoPtr, irMath::SE3 > > _GeometryList; ///< Geometry를 저장하고 있는 list 변수
		};
	}
}