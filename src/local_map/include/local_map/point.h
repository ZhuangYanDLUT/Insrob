#ifndef LOCAL_MAP_POINT_H
#define LOCAL_MAP_POINT_H
class CPoint3d
{
public:
	float x;	
	float y;	
	float z;	

public:
	
	CPoint3d(void);
	~CPoint3d(void);
	CPoint3d(const CPoint3d& tmpP);

};   
#endif  // LOCAL_MAP_POINT_H
