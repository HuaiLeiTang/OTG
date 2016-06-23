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
			/// Geometry type�� ����
			enum GEOMETRY_TYPE
			{
				_BOX,		///< Box ����
				_SPHERE,	///< Sphere ��
				_CAPSULE,	///< Capsule ĸ��
				_CYLINDER,	///< Cylinder ����
				_MESH,		///< Mesh mesh�� ������ ����
				_USERMODEL	///< UserModel ����ڰ� ���� ����� ��
			};

			/// ������
			GeometryInfo(const GEOMETRY_TYPE& Type, ///< Type
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : _Type(Type), _T(T), _Color(Color) {}

			/// �Ҹ���, ���� Ŭ������ ����� �ݴϴ�.
			virtual ~GeometryInfo() = 0;

			/// Type�� �����մϴ�.
			void setType(const GEOMETRY_TYPE& Type)
			{
				_Type = Type;
			}
			/// Frame�� ��ġ�� �����մϴ�.
			void setFrame(const irMath::SE3& T)
			{
				_T = T;
			}
			/// (R, G, B, Alpha) ���� �����մϴ�.
			void setColor(const irMath::Real& R, ///< Red
				const irMath::Real& G, ///< Green
				const irMath::Real& B, ///< Blue
				const irMath::Real& Alpha = (1.0) /// Alpha
				)
			{
				_Color << R, G, B, Alpha;
			}
			/// (R, G, B, Alpha) ���� �����մϴ�.
			void setColor(const irMath::Vector4& Color // �� [R; G; B; Alpha]
				)
			{
				_Color = Color;
			}

			/// ���� geometry�� type�� ������ �ɴϴ�.
			const GEOMETRY_TYPE& getType() const
			{
				return _Type;
			}
			/**
			*	\return irMath::Vector4 [R; G; B; Alpha]
			*	\brief ���� ���� �Ǿ��ִ� ���� �˷��ݴϴ�.
			*/
			const irMath::Vector4& getColor() const
			{
				return _Color;
			}
			/**
			*	\return irMath::SE3 Frame
			*	\brief ���� ���� �Ǿ� �ִ� Frame�� �˷��ݴϴ�.
			*/
			const irMath::SE3& getTransform() const
			{
				return _T;
			}

			/// Geometry ���� ����, �ڽ� Ŭ������ ������ copy�Լ� ������ �ʿ��մϴ�.
			virtual GeometryInfoPtr copy() const = 0;

		private:
			GEOMETRY_TYPE _Type; ///< Geometry type�� �����ϴ� ����
			irMath::SE3 _T; ///< Frame�� ��ġ�� �����ϰ� �ִ� ����
			irMath::Vector4 _Color; ///< R, G, B, Alpha ���� �����ϴ� ����

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		/**
		*	\class Box
		*	\brief Geometry box�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _BOX
		*/
		class Box :public GeometryInfo
		{
		public:
			/// �⺻ ������, ����=0, ����=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Box(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(0.0), _height(0.0), _depth(0.0) {}
			/// �Է����� ���� ����, ����, ���̷� �ʱ�ȭ�˴ϴ�.
			Box(const irMath::Real& width, ///< ����
				const irMath::Real& depth, ///< ����
				const irMath::Real& height, ///< ����
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(width), _height(height), _depth(depth) {}
			/// �Է����� ���� config�� �̿��Ͽ� �ʱ�ȭ�� �մϴ�.
			Box(const irMath::Vector3& config, ///< [����; ����; ����]
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(config(0)), _height(config(1)), _depth(config(2)) {}

			/// �Ҹ���
			~Box() {}

			/// �Է����� ���� ����, ����, ���̷� ���� �ٲߴϴ�.
			void setDimension(const irMath::Real& width, ///< ����
				const irMath::Real& depth, ///< ����
				const irMath::Real& height ///< ����
				);
			/// �Է����� ���� config�� ����, ����, ���̸� �ٲ��ݴϴ�.
			void setDimension(const irMath::Vector3& config ///< [����; ����; ����]
				);
			/// �Է����� ���� ���� �Ѻ��� ���̷� �ϴ� ������ü�� ����ϴ�.
			void setCube(const irMath::Real& length ///< ������ü�� �Ѻ��� ����
				);

			/**
			*	\return irMath::Vector3 [����; ����; ����]
			*	\brief ���� ���� �Ǿ��ִ� ����, ����, ���̸� �˷��ݴϴ�.
			*/
			irMath::Vector3 getDimension() const;

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _width, _depth, _height;
		};

		/**
		*	\class Sphere
		*	\brief Geometry sphere�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _SPHERE
		*/
		class Sphere :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0�� �ʱ�ȭ�˴ϴ�.
			Sphere(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(0.0) {}
			/// �Է����� ���� ���������� �ʱ�ȭ�˴ϴ�.
			Sphere(const irMath::Real& radius, ///< ������
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(radius) {}

			/// �Ҹ���
			~Sphere() {}

			/// �Է����� ���� radius�� ������ ���� �ٲߴϴ�.
			void setRadius(const irMath::Real& radius ///< ������
				);

			/**
			*	\return ������
			*	\brief ���� ���� �Ǿ��ִ� ������ ���� �˷��ݴϴ�.
			*/
			const irMath::Real& getRadius() const;

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius;
		};

		/**
		*	\class Capsule
		*	\brief Geometry capsule�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _CAPSULE
		*/
		class Capsule :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Capsule(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(0.0), _height(0.0) {}
			/// �Է����� ���� �������� ���̷� �ʱ�ȭ�˴ϴ�.
			Capsule(const irMath::Real& radius, ///< ������
				const irMath::Real& height, ///< ����
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(radius), _height(height) {}

			/// �Ҹ���
			~Capsule() {}

			/// �Է����� ���� ������, ���̷� ���� �ٲߴϴ�.
			void setDimension(const irMath::Real& radius, ///< ����
				const irMath::Real& height ///< ����
				);
			/// �Է����� ���� config�� ������, ���̸� �ٲ��ݴϴ�.
			void setDimension(const irMath::Vector2& config ///< [������: ����]
				);

			/**
			*	\return irMath::Vector2 [������; ����]
			*	\brief ���� ���� �Ǿ��ִ� ������, ���̸� �˷��ݴϴ�.
			*/
			irMath::Vector2 getDimension() const;

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius, _height;
		};

		/**
		*	\class Cylinder
		*	\brief Geometry cylinder�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _CYLINDER
		*/
		class Cylinder :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Cylinder(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(0.0), _height(0.0) {}
			/// �Է����� ���� �������� ���̷� �ʱ�ȭ�˴ϴ�.
			Cylinder(const irMath::Real& radius, ///< ������
				const irMath::Real& height, ///< ����
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(radius), _height(height) {}

			/// �Ҹ���
			~Cylinder() {}

			/// �Է����� ���� ������, ���̷� ���� �ٲߴϴ�.
			void setDimension(const irMath::Real& radius, ///< ����
				const irMath::Real& height ///< ����
				);
			/// �Է����� ���� config�� ������, ���̸� �ٲ��ݴϴ�.
			void setDimension(const irMath::Vector2& config ///< [������: ����]
				);

			/**
			*	\return irMath::Vector2 [������; ����]
			*	\brief ���� ���� �Ǿ��ִ� ������, ���̸� �˷��ݴϴ�.
			*/
			irMath::Vector2 getDimension() const;

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			irMath::Real _radius, _height;
		};

		/**
		*	\class Mesh
		*	\brief Geometry mesh�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _MESH
		*/
		class Mesh :public GeometryInfo
		{
		public:
			/// �⺻ ������, �ּҴ� NULL�� �ʱ�ȭ �˴ϴ�,
			Mesh(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url("") {}
			/// Mesh ������ ����Ǿ� �ִ� �ּ� ���� �̿��Ͽ� �ʱ�ȭ�մϴ�.
			Mesh(const std::string& url, ///< Mesh ������ ����Ǿ� �ִ� �ּ�
				const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url(url) {}

			/// Mesh ������ ����Ǿ� �ִ� �ּ� ���� �޾� �����մϴ�.
			void setUrl(const std::string& url ///< Mesh ������ ����Ǿ� �ִ� �ּ�
				)
			{
				_url = url;
			}

			/// ���� ���� �Ǿ��ִ� �ּ� ���� �˷��ݴϴ�.
			const std::string& getUrl() const
			{
				return _url;
			}

			void		setDimension(irMath::Real scale) { _scale = scale; }
			irMath::Real	getDimension() const { return _scale; }

			/**
			*	\return �����ϸ� True, �������� ������ False
			*	\brief ���� ���� �Ǿ��ִ� �ּҿ� ������ �����ϴ��� �Ǵ�
			*/
			bool isValid() const;

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			std::string _url;
			irMath::Real	_scale = 1;
		};

		/**
		*	\class UserModel
		*	\brief Geometry ������ ����ڰ� ������� ���� �� �ִ� Ŭ����, GEOMETRY_TYPE = _USERMODEL
		*	\todo addList ���ѷ��� ���� ���� �ʰ� �ϱ�, �˻�, ���� �Լ� �����
		*/
		class UserModel :public GeometryInfo
		{
		public:
			/// ������
			UserModel(const irMath::SE3& T = (irMath::SE3()), ///< Frame��ġ
				const irMath::Vector4& Color = (irMath::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_USERMODEL, T, Color), _GeometryList() {}

			/// srGeometry�ּҿ� irMath::SE3���� �޾Ƽ� �����մϴ�.
			void addList(const GeometryInfoPtr& shape, const irMath::SE3& T);

			/// List�κ��� ���� ã�ų� ������ �ٲٰ� ���� �� ����մϴ�.
			std::list< std::pair< GeometryInfoPtr, irMath::SE3 > >& getList()
			{
				return _GeometryList;
			}

			// ���� ����
			GeometryInfoPtr copy() const;

		private:
			std::list< std::pair< GeometryInfoPtr, irMath::SE3 > > _GeometryList; ///< Geometry�� �����ϰ� �ִ� list ����
		};
	}
}