LATEST_VERSION=v0.0.3

docker build --no-cache -t medusa_base_jenkins:${LATEST_VERSION} .
docker tag medusa_base_jenkins:${LATEST_VERSION} medusa_base_jenkins:latest

# Login at harbor.dsor
docker login harbor.dsor.isr.tecnico.ulisboa.pt

# Add the version and latest tags to harbor
docker tag medusa_base_jenkins:${LATEST_VERSION} harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:${LATEST_VERSION}
docker push harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:${LATEST_VERSION}
docker tag medusa_base_jenkins:latest harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest
docker push harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest