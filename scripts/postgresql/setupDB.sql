create role psqluser with login;

create database test_postgres with owner psqluser;

\c test_postgres psqluser;

create table shopping_list(
    nome varchar(32),
    prodotto varchar(32),
    quantità INTEGER,
    PRIMARY KEY (nome, prodotto)
);


insert into shopping_list (nome, prodotto, quantità) values('Rossi', 'Book', 12);
