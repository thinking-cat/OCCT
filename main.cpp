#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include "Viewer.h"

#include <Standard.hxx>
#include <Standard_Type.hxx>

#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BSplineSurface.hxx>

#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>

#include <BRepBuilderAPI.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <BRepAlgoAPI_Algo.hxx>
#include <BRepAlgoAPI_Fuse.hxx>

#include <BRepFilletAPI_MakeFillet.hxx>

#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>

#include <BRepLib.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>

#include <GCE2d_MakeSegment.hxx>

#include <TColgp_Array1OfPnt2d.hxx>
#include <TCollection_AsciiString.hxx>



#include <iostream>
#include <fstream>
#include <vector>
#include <utility>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;

typedef std::tuple<Point, int> P;
typedef CGAL::Nth_of_tuple_property_map<0, P> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, P> Index_map;

TopoDS_Shape makeBottle(Standard_Real myHeight, Standard_Real myWidth, Standard_Real myThickness);


int main() {
	
	// Get surface points
	std::ifstream surfaceFile("../PLY/face_points.ply");
	std::vector<P> face_points;
	if (!CGAL::IO::read_PLY_with_properties(
		surfaceFile, std::back_inserter(face_points),
		CGAL::IO::make_ply_point_reader(Point_map()),
		std::make_pair(Index_map(), CGAL::IO::PLY_property<int>("primitive_index"))
	)) {
		std::cerr << "Failed to open PLY file!" << std::endl;
		return EXIT_FAILURE;
	}

	// Get edge points
	std::ifstream edgeFile("../PLY/edge_points.ply");
	std::vector<P> edge_points;
	if (!CGAL::IO::read_PLY_with_properties(
		edgeFile, std::back_inserter(edge_points),
		CGAL::IO::make_ply_point_reader(Point_map()),
		std::make_pair(Index_map(), CGAL::IO::PLY_property<int>("primitive_index"))
	)) {
		std::cerr << "Failed to open PLY file!" << std::endl;
		return EXIT_FAILURE;
	}

	// Convert them to gp_Pnt
	std::vector<std::vector<gp_Pnt>> BSplineSurfacePoles(3);
	std::vector<std::vector<gp_Pnt>> BSplineEdgePoles(3);
	for (int i = 0; i < face_points.size(); i++) {
		const Point& surfacePoint = get<0>(face_points[i]);
		const int surfaceInd = get<1>(face_points[i]);

		BSplineSurfacePoles[surfaceInd].push_back(gp_Pnt(surfacePoint.x(), surfacePoint.y(), surfacePoint.z()));
	}

	for (int i = 0; i < edge_points.size(); i++) {
		const Point& edgePoint = get<0>(edge_points[i]);
		const int edgeInd = get<1>(edge_points[i]);

		BSplineEdgePoles[edgeInd].push_back(gp_Pnt(edgePoint.x(), edgePoint.y(), edgePoint.z()));
	}


	// test
	Viewer vout(100, 100, 1080, 960);
	//const TopoDS_Shape shape = makeBottle(70.0, 50.0, 30.0);
	/*if (shape.IsNull()) {
		std::cout << "Null" << std::endl;
		return 1;
	}

	vout << shape;
	vout.StartMessageLoop();*/
	std::vector<Handle(Geom_BSplineSurface)> BSSurfaces;
	for (int ind = 0; ind < 3; ind++) {
		TColgp_Array2OfPnt controlPointGrid(1, 20, 1, 20);
		for (int i = 1; i <= 20; ++i) {
			for (int j = 1; j <= 20; ++j) {
				controlPointGrid.SetValue(i, j, BSplineSurfacePoles[ind][(i - 1) * 20 + j - 1]);
			}
		}
		TColStd_Array1OfReal UKnots(1, 20 + 1 + 3 + 1);
		TColStd_Array1OfReal VKnots(1, 20 + 1 + 3 + 1);

		for (int i = 1; i <= 25; i++) {
			UKnots.SetValue(i, i - 1);
			VKnots.SetValue(i, i - 1);
		}

		TColStd_Array1OfInteger UMults(1, 20 + 1 + 3 + 1);
		TColStd_Array1OfInteger VMults(1, 20 + 1 + 3 + 1);

		for (int i = 1; i <= 25; i++) {
			UMults.SetValue(i, 1);
			VMults.SetValue(i, 1);
		}

		Handle(Geom_BSplineSurface) aBSplineSurface = new Geom_BSplineSurface(
			controlPointGrid,
			UKnots,
			VKnots,
			UMults, // Degree in U direction
			VMults, // Degree in V direction
			3, // U Degree 
			3 // V Degree
		);

		BSSurfaces.push_back(aBSplineSurface);
	}

	TopoDS_Compound result;
	BRep_Builder mkCompound;
	mkCompound.MakeCompound(result);
	//std::vector<TopoDS_Face> TopoBSsurfaces;
	for (int i = 0; i < 3; i++) {
		TopoDS_Face aSurface = BRepBuilderAPI_MakeFace(BSSurfaces[0], Precision::Confusion());
		mkCompound.Add(result, aSurface);
	}

	vout << result;
	
	
	std::cout << "End" << std::endl;
	return 0;
}






TopoDS_Shape makeBottle(Standard_Real myHeight, Standard_Real myWidth, Standard_Real myThickness) {
	// Define the half of the points on the bottom wire of Bottle 
	gp_Pnt aPnt1(-myWidth / 2.0, 0.0, 0.0);
	gp_Pnt aPnt2(-myWidth / 2.0, -myThickness / 4.0, 0.0);
	gp_Pnt aPnt3(0.0, -myThickness / 2.0, 0.0);
	gp_Pnt aPnt4(myWidth / 2.0, -myThickness / 4.0, 0.0);
	gp_Pnt aPnt5(myWidth / 2.0, 0.0, 0.0);

	// Create Geometric edges
	Handle(Geom_TrimmedCurve) aArcOfCircle = GC_MakeArcOfCircle(aPnt2, aPnt3, aPnt4);
	Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPnt1, aPnt2);
	Handle(Geom_TrimmedCurve) aSegment2 = GC_MakeSegment(aPnt4, aPnt5);

	// Create Topological edges
	TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);
	TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(aArcOfCircle);
	TopoDS_Edge anEdge3 = BRepBuilderAPI_MakeEdge(aSegment2);

	// Connect edges to wire
	TopoDS_Wire aWire = BRepBuilderAPI_MakeWire(anEdge1, anEdge2, anEdge3);

	// Create transformation
	const gp_Ax1 xAxis = gp::OX();
	gp_Trsf aTrsf;
	aTrsf.SetMirror(xAxis);

	// Apply transformation to create a mirro wire
	BRepBuilderAPI_Transform aBrepTransform(aWire, aTrsf);
	const TopoDS_Shape aMirroShape = aBrepTransform.Shape();
	TopoDS_Wire aMirroWire = TopoDS::Wire(aMirroShape);
	
	// Connect the wires
	BRepBuilderAPI_MakeWire mkWire;
	mkWire.Add(aWire);
	mkWire.Add(aMirroWire);
	TopoDS_Wire myWireProfile = mkWire.Wire();

	// The wire can bound a face
	TopoDS_Face myFaceProfile = BRepBuilderAPI_MakeFace(myWireProfile);

	// Define a vector to construct the body of the bottle using the bottom face
	gp_Vec aPrismVec(0.0, 0.0, myHeight);

	// Using Prism to create the body of the bottle
	TopoDS_Shape myBody = BRepPrimAPI_MakePrism(myFaceProfile, aPrismVec);

	// Fillet the body of the bottle, which needs to explicitly add the edge
	BRepFilletAPI_MakeFillet mkFillet(myBody);

	// Iterate all the edges of the body, then make fillet
	TopExp_Explorer anEdgeExplorer(myBody, TopAbs_EDGE);
	while (anEdgeExplorer.More()) {
		//get the edge
		const TopoDS_Edge edge = TopoDS::Edge(anEdgeExplorer.Current());
		//adding to BRepFilletAPI_MakeFillet function to fillet
		mkFillet.Add(myThickness / 12.0, edge);
		anEdgeExplorer.Next();
	}

	// Get the filleted body shape
	myBody = mkFillet.Shape();
	
	// Create the bottle neck
	gp_Pnt neckLocation(0.0, 0.0, myHeight);
	// The Difference between gp_Dir and gp_Ax is the former doesn't have a start point(location) 
	// while the latter has one and the Ax means the axis, which indicates not only the XYZ axises, 
	// but the axis of some TopoDS_Shape in the 3D space.
	gp_Dir neckAxis = gp::DZ();
	gp_Ax2 neckAx2(neckLocation, neckAxis);
	
	// Create the neck of the bottle
	Standard_Real myNeckRadius = myThickness / 4.0;
	Standard_Real myNeckHeight = myHeight / 10.0;
	BRepPrimAPI_MakeCylinder MKCylinder(neckAx2, myNeckRadius, myNeckHeight);
	TopoDS_Shape neck = MKCylinder.Shape();

	// Combine the body and the neck
	myBody = BRepAlgoAPI_Fuse(myBody, neck);

	// Find the upmost surface, which specifically is a *plane*
	TopoDS_Face toRemove;
	Standard_Real zMax = -1;
	for (TopExp_Explorer deleteHighestplane(myBody, TopAbs_FACE); deleteHighestplane.More(); deleteHighestplane.Next()) {
		TopoDS_Face aFace = TopoDS::Face(deleteHighestplane.Current());
		Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aFace);
		if (aSurface->DynamicType() == STANDARD_TYPE(Geom_Plane)) {
			Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(aSurface);
			if (aPlane->Location().Z() > zMax) {
				zMax = aPlane->Location().Z();
				toRemove = aFace;
			}
		}
	}

	// Since the function BRepOffsetAPI_MakeThickSolid can take a list of shapes to remove as an argument,
	// so we append the hightest plane, which obviously is the mouth of the bottle.
	TopTools_ListOfShape facesToRemove;
	facesToRemove.Append(toRemove);

	BRepOffsetAPI_MakeThickSolid mkThickSolid;
	mkThickSolid.MakeThickSolidByJoin(myBody, facesToRemove, -myThickness / 50.0, 1.e-3);
	myBody = mkThickSolid.Shape();

	// Threading : Create Surfaces
	Handle(Geom_CylindricalSurface) aCyl1 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 0.99);
	Handle(Geom_CylindricalSurface) aCyl2 = new Geom_CylindricalSurface(neckAx2, myNeckRadius * 1.05);

	// Threading : Define 2D Curves
	gp_Pnt2d aPnt(2. * M_PI, myNeckHeight / 2.);
	gp_Dir2d aDir(2. * M_PI, myNeckHeight / 4.);
	gp_Ax2d anAx2d(aPnt, aDir);

	Standard_Real aMajor = 2. * M_PI;
	Standard_Real aMinor = myNeckHeight / 10;

	// Defining the ellipse to get the trimmed curve
	Handle(Geom2d_Ellipse) anEllipse1 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor);
	Handle(Geom2d_Ellipse) anEllipse2 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4.0);

	// Get the trimmed curve
	Handle(Geom2d_TrimmedCurve) anArc1 = new Geom2d_TrimmedCurve(anEllipse1, 0.0, M_PI);
	Handle(Geom2d_TrimmedCurve) anArc2 = new Geom2d_TrimmedCurve(anEllipse2, 0.0, M_PI);

	// But these are just the curves, we still need a line segment to connect these two curves seperately
	gp_Pnt2d startPnt;
	gp_Pnt2d endPnt;
	anEllipse1->D0(0, startPnt);
	anEllipse1->D0(M_PI, startPnt);
	Handle(Geom2d_TrimmedCurve) aSegment = GCE2d_MakeSegment(startPnt, endPnt);
	
	// Now create the TopoDS_Edge(caution: these are curves defining on 2D space)
	TopoDS_Edge edge1OnSurface1 = BRepBuilderAPI_MakeEdge(anArc1, aCyl1);
	TopoDS_Edge edge2OnSurface1 = BRepBuilderAPI_MakeEdge(aSegment, aCyl1);
	TopoDS_Edge edge1OnSurface2 = BRepBuilderAPI_MakeEdge(anArc2, aCyl2);
	TopoDS_Edge edge2OnSurface2 = BRepBuilderAPI_MakeEdge(aSegment, aCyl2);

	TopoDS_Wire threadWire1 = BRepBuilderAPI_MakeWire(edge1OnSurface1, edge2OnSurface1);
	TopoDS_Wire threadWire2 = BRepBuilderAPI_MakeWire(edge1OnSurface2, edge2OnSurface2);

	// Because these wires are built out of 2D(Cylinder surface belongs to 2D surface as well), 
	// there's no information about 3D curves and we want to build the threads of the bottle neck
	// with 3D curves, we must transform it into a 3D curve.
	BRepLib::BuildCurves3d(threadWire1);
	BRepLib::BuildCurves3d(threadWire2);

	// By default, this tool just build a shell, so we must set it true
	BRepOffsetAPI_ThruSections BuildThreads(Standard_True);
	BuildThreads.AddWire(threadWire1);
	BuildThreads.AddWire(threadWire2);
	// Close the check for the number of edges so that they are the same
	BuildThreads.CheckCompatibility(Standard_False);
	TopoDS_Shape myThreads = BuildThreads.Shape();

	// return the result, which is a TopoDS_Compound
	TopoDS_Compound result;
	BRep_Builder builder;
	builder.MakeCompound(result);
	builder.Add(result, myBody);
	builder.Add(result, myThreads);

	return result;
}