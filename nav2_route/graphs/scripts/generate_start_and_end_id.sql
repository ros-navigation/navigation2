WITH start_points AS
(
	SELECT  n.id       AS node_id
	       ,e.id       AS edge_id
	       ,e.geometry AS edge_geometry
	FROM edges AS e, nodes AS n
	WHERE ST_INTERSECTS(n.geometry, ST_StartPoint(e.geometry))
), end_points AS
(
	SELECT  n.id       AS node_id
	       ,e.id       AS edge_id
	       ,e.geometry AS edge_geometry
	FROM edges AS e, nodes AS n
	WHERE ST_INTERSECTS(n.geometry, ST_EndPoint(e.geometry))
)
SELECT  sp.edge_id       AS edge_id
       ,sp.node_id       AS start_node
       ,ep.node_id       AS end_node
       ,sp.edge_geometry AS geometry
FROM start_points AS sp
JOIN end_points AS ep
ON sp.edge_id = ep.edge_id
