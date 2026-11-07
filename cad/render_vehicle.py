"""Render the generated CAD assembly with VTK (not an artist impression)."""
import argparse
import json
from pathlib import Path

from vtkmodules import all as vtk
from mechanical_vehicle import parts, mounts, COLORS


def main():
    root=Path(__file__).resolve().parent.parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=root/'cad/preview.png')
    args=parser.parse_args()
    d=json.loads((root/'src/ackermann_description/config/vehicle.json').read_text())
    renderer=vtk.vtkRenderer();renderer.SetBackground(.92,.94,.97)
    origins=mounts(d)
    for p in parts(d):
        positions=[origins[p.group]] if p.group!='wheel' else [
            (x,y,d['wheel_radius']*1000) for x in (0,d['wheelbase']*1000)
            for y in (-d['track_width']*500,d['track_width']*500)]
        vertices,faces=p.shape.tessellate(.5,.15)
        points=vtk.vtkPoints()
        for v in vertices:points.InsertNextPoint(v.x,v.y,v.z)
        cells=vtk.vtkCellArray()
        for face in faces:
            cells.InsertNextCell(3)
            for v in face:cells.InsertCellPoint(v)
        poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(cells)
        normals=vtk.vtkPolyDataNormals();normals.SetInputData(poly);normals.SetFeatureAngle(35)
        mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(normals.GetOutputPort())
        for position in positions:
            actor=vtk.vtkActor();actor.SetMapper(mapper);actor.SetPosition(*position)
            actor.GetProperty().SetColor(*COLORS[p.material]);actor.GetProperty().SetSpecular(.25)
            renderer.AddActor(actor)
    camera=renderer.GetActiveCamera();camera.SetPosition(730,-820,660)
    camera.SetFocalPoint(d['wheelbase']*500,0,65);camera.SetViewUp(0,0,1)
    camera.ParallelProjectionOn();camera.SetParallelScale(270)
    window=vtk.vtkRenderWindow();window.AddRenderer(renderer);window.SetSize(1400,1000)
    window.SetOffScreenRendering(1);renderer.ResetCameraClippingRange();window.Render()
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(args.output));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
    window.Finalize();print(args.output)


if __name__=='__main__':main()
