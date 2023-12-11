# SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
# SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
#
# SPDX-License-Identifier: BSD-3-Clause

DOCKERHUB=sgl-artifactory.arlut.utexas.edu/artifactory/docker
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
mkfile_dir := $(dir $(mkfile_path))

py3env-new/bin/activate:
	virtualenv --system-site-packages -p python3 py3env-new

py3env-new/modules: py3env-new/bin/activate
	mkdir -p py3env-new/modules

py3env-new/bin/cocotb-config: py3env-new/bin/activate py3env-new/modules
	. py3env-new/bin/activate; \
	pip3 install cocotb; \
	pip3 install cocotb-bus; \
	pip3 install pytest; \
	cd py3env-new/modules; \
	git clone https://sgl-git.arlut.utexas.edu/slicer/externals/cocotbext-i2c.git; \
	pip3 install -e cocotbext-i2c; \
	git clone https://sgl-git.arlut.utexas.edu/slicer/eposhh_fir_sw.git; \
	pip3 install -e eposhh_fir_sw; \
	deactivate

ENV_PRODUCTS=py3env-new/bin/cocotb-config
env: $(ENV_PRODUCTS)

sim: env
	cd sim; make


vformat: lint/verible-verilog-format.rules
	docker run -w $(mkfile_dir) -v $(HOME):$(HOME) $(DOCKERHUB)/hdlc/verible:latest verible-verilog-format --inplace --flagfile lint/verible-verilog-format.rules rtl/*.v

vlint: lint/verible-verilog-lint.rules
	docker run -w $(mkfile_dir) -v $(HOME):$(HOME) $(DOCKERHUB)/hdlc/verible:latest verible-verilog-lint rtl/*.v --rules_config lint/verible-verilog-lint.rules --waiver_files lint/verible-verilog-lint.waiver
clean:
	rm -rf py3env-new
