from py2neo import Graph, Node, Relationship

g = Graph(host="localhost") #password="admin")

tx = g.begin()
john = Node("User", name="John", gender="M", age="28")
tx.create(john)

mary = Node("User", name="Mary", gender="F", age="26")
tx.create(mary)

jm = Relationship(john, "FRIENDS_WITH", mary)
tx.create(jm)

tx.commit()