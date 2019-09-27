function obj = Table(xOffset,yOffset,zOffset)

[f,v,data] = plyread('table.ply');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
obj = trisurf(f,v(:,1)+ xOffset, ...
                v(:,2)+ yOffset, ...
                v(:,3)+ zOffset, ...
    'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

end
