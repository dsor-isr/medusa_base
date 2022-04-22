docker build --no-cache -t medusa_base_jenkins:v0.0.1 .
docker tag medusa_base_jenkins:v0.0.1 medusa_base_jenkins:latest

# Login at harbor.dsor
docker login harbor.dsor.isr.tecnico.ulisboa.pt

# Add the version and latest tags to harbor
docker tag medusa_base_jenkins:v0.0.1 harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.1
docker push harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.1
docker tag medusa_base_jenkins:latest harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest
docker push harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest