# Simulação e Controle de Braço Robótico 2DOF

A atividade consistia em criar um braço robótico controlador PID com 2 a 3 juntas que reagisse a pertubações no ambiente externo.

Criei um arquivo URDF que cria uma garra de duas juntas, uma que rotaciona e outra que é feita para se mover na vertical, após isso adicionei no código um controle de pid manual que reage as teclas "i", "j", "k" e "l", onde ao clicar na tecla é calculado o angulo atual da junta que vai sofrer a pertubação e é adicionado um torque a essa junta após isso, pode ver isso das linhas 234 até a linha 243.

Adicionei 3 blocos de pesos diferentes que são ativados ao clicar nas teclas "1", "2" e "3" para provar que a garra reage aos pesos de maneira diferente, onde o mais leve não impacta na movimentação da garra, o médio impacta medianamente dificultando o controle e locomação e o mais pesado torna a garra quase que imóvel, pode ver a adição dos blocos nas linhas 290 à 293 e 106 à 148.
